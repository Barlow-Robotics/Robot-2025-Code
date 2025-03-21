// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class DynamicAutoBuilder {

  Drive driveSub;
  Vision visionSub;
  Joystick driverController;

  PathConstraints constraints = new PathConstraints(2, 6.0, 6 * Math.PI, 12 * Math.PI);
  Transform2d centeredOffset = new Transform2d(Constants.FieldConstants.reefOffsetMeters, 0.0, Rotation2d.k180deg);

  private final ProfiledPIDController profiledPIDX ;
  private final ProfiledPIDController profiledPIDY ;
  private final ProfiledPIDController profiledPIDRot ;

  private final SimpleMotorFeedforward feedForwardX ;
  private final SimpleMotorFeedforward feedForwardY ;
  private final SimpleMotorFeedforward feedForwardRot ;


  public DynamicAutoBuilder(Drive d, Vision v, Joystick j) {
    driveSub = d;
    visionSub = v;
    driverController = j;

    Transform2d centeredOffset = new Transform2d(Constants.FieldConstants.reefOffsetMeters, 0.0, Rotation2d.k180deg);

    profiledPIDX = new ProfiledPIDController(0, 0, 0, null) ;
    profiledPIDY = new ProfiledPIDController(0, 0, 0, null) ;
    profiledPIDRot = new ProfiledPIDController(0, 0, 0, null) ;

    feedForwardX = new SimpleMotorFeedforward(0, 1.0 / Constants.VisionConstants.AutoAlignVelocityConstant) ;
    feedForwardY = new SimpleMotorFeedforward(0, 1.0 / Constants.VisionConstants.AutoAlignVelocityConstant) ;
    feedForwardRot = new SimpleMotorFeedforward(0, 1.0 / Constants.VisionConstants.AutoAlignAngularVelocityConstant) ;


  }

  public Command pathPlannerAlign(Transform2d extraOffset) {
    // pathPlannerAlign uses a deferred so that it can compute the new path when the
    // trigger is run.
    return new DeferredCommand(
        () -> computeAutoPathPlanner(extraOffset),
        Set.of(driveSub));
  }

  public Command manualAlign(Transform2d extraOffset) {
    // manualAlign uses a deferred so that it can compute the new path when the
    // trigger is run.
    return new DeferredCommand(
        () -> runManualPathing(extraOffset),
        Set.of(driveSub));
  }

  public Command trapazoidAlign(Transform2d extraOffset) {
    // manualAlign uses a deferred so that it can compute the new path when the
    // trigger is run.
    return new DeferredCommand(
        () -> runTrapazoidPathing(extraOffset),
        Set.of(driveSub));
  }



  // extraOffset specifies the additional offset for the target pose
  Command computeAutoPathPlanner(Transform2d extraOffset) {

    var maybeTargetPose = findTargetFromOffset(extraOffset);
    if (maybeTargetPose.isEmpty()) {
      return Commands.none();
    }

    var targetPose = maybeTargetPose.get();

    var currentPose = driveSub.getPose();

    // Create our waypoints
    var waypoints = PathPlannerPath.waypointsFromPoses(
        currentPose,
        // Insert an extra waypoint half a meter back so that we always go in straight
        targetPose.transformBy(new Transform2d(-0.5, 0, Rotation2d.kZero)),
        targetPose);

    // Plan a path along the waypoints subject to the constriants
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, targetPose.getRotation()));

    // If the path is broken (only 1 point), prevents crashing.
    if (path.getAllPathPoints().size() < 2) {
      return Commands.none();
    }

    return AutoBuilder.followPath(path);
  }

  Command applyDeltaRequest(Pose2d targetPose) {
    return Commands.run(() -> {
      // The translation delta is the amount we would need to add to our current
      // drive pose to get the robot to end at the targetPose.
      // targetPose = drivePose + (targetPose - drivePose)
      // Commanding this as a veelocity moves us in the direction of the target.
      Translation2d translationDelta = (targetPose.getTranslation()).minus(driveSub.getPose().getTranslation());
      double rotationDelta = targetPose.getRotation().minus(driveSub.getPose().getRotation()).getRadians();

      // Always use BlueAlliance for the ForwardPerspective value. This is because
      // all of our poses are always relative to blue-alliance.
      FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withVelocityX(translationDelta.getX())
          .withVelocityY(translationDelta.getY())
          .withRotationalRate(rotationDelta);

      double sliderInput = -driverController.getThrottle();
      double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;

      swerveRequest.VelocityX *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
      if (swerveRequest.VelocityX > 3) {
        swerveRequest.VelocityX = 3;
      }
      swerveRequest.VelocityY *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
      if (swerveRequest.VelocityY > 3) {
        swerveRequest.VelocityY = 3;
      }
      swerveRequest.RotationalRate *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier * 2;

      driveSub.setControl(swerveRequest);
    });
  }

  Command runManualPathing(Transform2d extraOffset) {

    var maybeTargetPose = findTargetFromOffset(extraOffset);
    if (maybeTargetPose.isEmpty()) {
      return Commands.none();
    }

    var targetPose = maybeTargetPose.get();

    return applyDeltaRequest(targetPose)
        .until(() -> driveSub.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05 &&
            Math.abs(driveSub.getPose().getRotation().minus(targetPose.getRotation()).getRadians()) < 0.01);
  }



  Command runTrapazoidPathing(Transform2d extraOffset) {

    var maybeTargetPose = findTargetFromOffset(extraOffset);
    if (maybeTargetPose.isEmpty()) {
      return Commands.none();
    }

    var targetPose = maybeTargetPose.get();

    return applyDeltaRequest(targetPose)
        .until(() -> driveSub.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05 &&
            Math.abs(driveSub.getPose().getRotation().minus(targetPose.getRotation()).getRadians()) < 0.01);
  }







  Command applyTrapazoidalRequest(Pose2d targetPose) {

    FunctionalCommand fc = new FunctionalCommand(
      ()->{
        // set goals in profiled PIDs
        profiledPIDX.setGoal(0);
      }, 
      () -> {
        // set velocities based on PID calculations and feed forward values
      }, 
      null, 
      null, 
      driveSub
      ) ;
    return fc ;

    // return Commands.run(() -> {
    //   // The translation delta is the amount we would need to add to our current
    //   // drive pose to get the robot to end at the targetPose.
    //   // targetPose = drivePose + (targetPose - drivePose)
    //   // Commanding this as a veelocity moves us in the direction of the target.
    //   Translation2d translationDelta = (targetPose.getTranslation()).minus(driveSub.getPose().getTranslation());
    //   double rotationDelta = targetPose.getRotation().minus(driveSub.getPose().getRotation()).getRadians();

    //   // Always use BlueAlliance for the ForwardPerspective value. This is because
    //   // all of our poses are always relative to blue-alliance.
    //   FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
    //       .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    //       .withVelocityX(translationDelta.getX())
    //       .withVelocityY(translationDelta.getY())
    //       .withRotationalRate(rotationDelta);

    //   double sliderInput = driverController.getThrottle();
    //   double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;

    //   swerveRequest.VelocityX *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
    //   swerveRequest.VelocityY *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
    //   swerveRequest.RotationalRate *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;

    //   Logger.recordOutput("AutoAlign/VelocityX", swerveRequest.VelocityX);
    //   Logger.recordOutput("AutoAlign/VelocityY", swerveRequest.VelocityY);
    //   Logger.recordOutput("AutoAlign/RotationalRate", swerveRequest.RotationalRate);

    //   driveSub.setControl(swerveRequest);
    // });
  }





  Optional<Pose2d> findTargetFromOffset(Transform2d extraOffset) {
    // Find the closet tag
    var currentPose = driveSub.getPose();
    var tagPose = findPoseOfTagClosestToRobot(currentPose);

    // If there wasn't one, do nothing
    if (tagPose.isEmpty()) {
      return Optional.empty();
    }

    var targetPose = tagPose.get().plus(centeredOffset).plus(extraOffset);

    Logger.recordOutput("Auto/Target", targetPose);

    // Adjust by the offsets
    return Optional.of(targetPose);
  }




  Optional<Pose2d> findPoseOfTagClosestToRobot(Pose2d drivePose) {

    int[] aprilTagList;
    var alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      return Optional.empty();
    }

    if (alliance.get() == DriverStation.Alliance.Blue) {
      aprilTagList = Constants.VisionConstants.blueAprilTagListReef;
    } else if (alliance.get() == DriverStation.Alliance.Red) {
      aprilTagList = Constants.VisionConstants.redAprilTagListReef;
    } else {
      // Short circuit so we don't end up with an empty list
      return Optional.empty();
    }

    List<Pose2d> possiblePoses = new ArrayList<>();

    for (int id : aprilTagList) {
      possiblePoses.add(visionSub.getLayout().getTagPose(id).get().toPose2d());
    }

    return Optional.of(drivePose.nearest(possiblePoses));
  }
}

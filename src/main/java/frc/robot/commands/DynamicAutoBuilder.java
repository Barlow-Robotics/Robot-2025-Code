// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class DynamicAutoBuilder {

  Drive driveSub;
  Vision visionSub;
  Joystick driverController;

  PathConstraints constraints = new PathConstraints(2, 6.0, 6 * Math.PI, 12 * Math.PI);
  Transform2d centeredOffset = new Transform2d(Constants.FieldConstants.reefOffsetMeters, 0.0, Rotation2d.k180deg);

  public DynamicAutoBuilder(Drive d, Vision v, Joystick j) {
    driveSub = d;
    visionSub = v;
    driverController = j;
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
      Translation2d translationDelta = (driveSub.getPose().getTranslation()).minus(targetPose.getTranslation());
      double rotationDelta = targetPose.getRotation().minus(driveSub.getPose().getRotation()).getRadians();

      FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
          .withVelocityX(translationDelta.getX())
          .withVelocityY(translationDelta.getY())
          .withRotationalRate(rotationDelta);

      double sliderInput = -driverController.getThrottle();
      double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;

      swerveRequest.VelocityX *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
      swerveRequest.VelocityY *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;
      swerveRequest.RotationalRate *= Constants.VisionConstants.AutoAlignVelocityConstant * maxVelocityMultiplier;

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

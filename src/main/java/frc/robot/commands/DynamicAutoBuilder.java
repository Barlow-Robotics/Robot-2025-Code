// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
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

  PathConstraints constraints = new PathConstraints(2, 6.0, 6 * Math.PI, 12 * Math.PI);
  Transform2d centeredOffset = new Transform2d(Constants.FieldConstants.reefOffsetMeters, 0.0, Rotation2d.k180deg);

  public DynamicAutoBuilder(Drive d, Vision v) {
    driveSub = d;
    visionSub = v;
  }

  public Command dynamicAuto(Transform2d extraOffset) {
    // dynamicAuto uses a deferred so that it can compute the new path when the
    // trigger is run.
    return new DeferredCommand(
        () -> computeAutoPath(extraOffset),
        Set.of(driveSub));
  }

  // extraOffset specifies the additional offset for the target pose
  Command computeAutoPath(Transform2d extraOffset) {
    // Find the closet tag
    var currentPose = driveSub.getPose();
    var tagPose = findPoseOfTagClosestToRobot(currentPose);

    // If there wasn't one, do nothing
    if (tagPose.isEmpty()) {
      return Commands.none();
    }

    // Adjust by the offsets
    var targetPose = tagPose.get().plus(centeredOffset).plus(extraOffset);
    Logger.recordOutput("Auto/Target", targetPose);

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

  Optional<Pose2d> findPoseOfTagClosestToRobot(Pose2d drivePose) {

    int[] aprilTagList;
    var alliance = DriverStation.getAlliance();

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElectronicsIDs;
import edu.wpi.first.math.geometry.Pose2d;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command currentTeleopCommand;

  private final RobotContainer robotContainer;
  boolean pathPlannerConfigured = false ;
  boolean currentlyFollowingAPath = false;
  Pose2d currentPose;
  public Robot() {
    robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "2025-Robot-Code"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter("/media/sda2/")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher());
        // Publish data to NetworkTables
        // CHANGE - leaks below
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        Logger.addDataReceiver(new WPILOGWriter(""));
        Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
  }

  @Override
  public void robotPeriodic() {
    String currentDriverController = DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort);
    String currentOperatorController = DriverStation.getJoystickName(ElectronicsIDs.OperatorControllerPort);
    Logger.recordOutput("Controllers/Driver", currentDriverController);
    Logger.recordOutput("Controllers/Operator", currentOperatorController);
    currentPose = robotContainer.driveSub.getPose();
    var poseEstimate = robotContainer.visionSub.getEstimatedGlobalPose(currentPose);
    if (poseEstimate.isPresent()) {
        Logger.recordOutput("vision/estimatedPose", poseEstimate.get().estimatedPose);
    }
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // System.err.println(robotContainer.getCoralVision());
    boolean test = true;
    if (robotContainer.getCoralVision()) { // button is pressed and I want to look for april tag and move with auto
      Command selectedAutoCommand = robotContainer.getVisionPathPlannerPathing();

      if (selectedAutoCommand != null) {
        System.out.println("PATH");
      }
      else {
        System.out.println("NO PATH");
        System.out.println(robotContainer.visionSub.getBestTrackableTarget());
      }

      if (!currentlyFollowingAPath && selectedAutoCommand != null) {
          currentlyFollowingAPath = true;
          currentTeleopCommand = selectedAutoCommand;
          System.out.println("SCEDUAL PROBLEM");
          CommandScheduler.getInstance().schedule(selectedAutoCommand);
          System.out.println("SCEDUAL PROBLEM");
      }
    }
    if (currentlyFollowingAPath == true && currentTeleopCommand != null && currentTeleopCommand.isFinished()) { // if finished tell currentlyFollowingAPath. 
        currentlyFollowingAPath = false;
        if (currentTeleopCommand != null) {
          currentTeleopCommand.cancel();
        }
        currentTeleopCommand = null;
    }
    if (currentlyFollowingAPath) {
        
        // check the driver controller that it hasnt moved too much. 
   }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command currentTeleopCommand;

  private final RobotContainer robotContainer;
  boolean pathPlannerConfigured = false ;
  boolean currentlyFollowingAPath = false;

  /*****  MECHANISM 2D FOR ADVANTAGE SCOPE  *****/

    // the main mechanism object
    Mechanism2d mech = new Mechanism2d(8, 10);

    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("manipulator", 2.5, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", ArmConstants.ElevatorMinimumHeight, 90));
    MechanismLigament2d arm = elevator.append(
            new MechanismLigament2d("arm", 2, 0, 6, new Color8Bit(Color.kPurple)));
    MechanismLigament2d gripper = arm.append(new MechanismLigament2d("gripper", .5, 10, 10, new Color8Bit(Color.kLimeGreen)));

    /**********************************************/

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
    robotContainer.visionSub.periodic();
    String currentDriverController = DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort);
    String currentOperatorController = DriverStation.getJoystickName(ElectronicsIDs.OperatorControllerPort);
    Logger.recordOutput("Controllers/Driver/CurrentController", currentDriverController);
    // Logger.recordOutput("Controllers/Driver/X_Value", RobotContainer.driverController.getRawAxis(LogitechExtreme3DConstants.AxisX));
    // Logger.recordOutput("Controllers/Driver/Y_Value", RobotContainer.driverController.getRawAxis(LogitechExtreme3DConstants.AxisY));
    // Logger.recordOutput("Controllers/Driver/Twist_Value", RobotContainer.driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate));
    Logger.recordOutput("Controllers/Operator/CurrentController", currentOperatorController);

    // Logger.recordOutput("RobotPose", new Pose2d());
    // Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    // Logger.recordOutput(
    //   "FinalComponentPoses", 
    //   new Pose3d[] {
    //     new Pose3d(
    //       -0.238, 0.0, 0.298, new Rotation3d(0.0, Math.sin(Timer.getTimestamp
    //       ())- 1.0, 0.0))});

    elevator.setLength(ArmConstants.ElevatorMinimumHeight + (robotContainer.armSub.getElevatorHeightInches() + robotContainer.armSub.getCarriageHeightInches())/12.0);
    arm.setAngle(robotContainer.armSub.getArmEncoderDegrees()-90);
    gripper.setAngle(robotContainer.armSub.getWristEncoderDegrees());

    SmartDashboard.putData("ArmMech2D", mech);

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
        // System.out.println("PATH");
      }
      else {
        // System.out.println("NO PATH");
        // System.out.println(robotContainer.visionSub.getBestTrackableTarget());
      }

      if (!currentlyFollowingAPath && selectedAutoCommand != null) {
          currentlyFollowingAPath = true;
          currentTeleopCommand = selectedAutoCommand;
          // System.out.println("SCHEDULE PROBLEM");
          CommandScheduler.getInstance().schedule(selectedAutoCommand);
          // System.out.println("SCHEDULE PROBLEM");
      }
    }
    if (currentlyFollowingAPath == true && currentTeleopCommand != null && currentTeleopCommand.isFinished()) { // if finished tell currentlyFollowingAPath. 
        currentlyFollowingAPath = false;
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

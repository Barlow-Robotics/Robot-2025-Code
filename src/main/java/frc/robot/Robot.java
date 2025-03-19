// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  boolean pathPlannerConfigured = false;
  Pose2d currentPose;

  /***** MECHANISM 2D FOR ADVANTAGE SCOPE *****/

  // the main mechanism object
  Mechanism2d mech = new Mechanism2d(8, 10);

  // the mechanism root node
  MechanismRoot2d root = mech.getRoot("manipulator", 2.5, 0);

  // MechanismLigament2d objects represent each "section"/"stage" of the
  // mechanism, and are based
  // off the root node or another ligament object
  MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", ArmConstants.ArmMinimumHeight, 90));
  MechanismLigament2d arm = elevator.append(
      new MechanismLigament2d("arm", 2, 0, 6, new Color8Bit(Color.kPurple)));
  MechanismLigament2d gripper = arm
      .append(new MechanismLigament2d("gripper", .5, 10, 10, new Color8Bit(Color.kLimeGreen)));

  /**********************************************/

  public Robot() {
    robotContainer = new RobotContainer(this);
    Logger.recordMetadata("ProjectName", "2025-Robot-Code"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.addDataReceiver(new NT4Publisher());
      // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging - ignore leak
    } else {
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
  }

  @Override
  public void robotInit() {
    SignalLogger.start();

  }

  @Override
  public void robotPeriodic() {
    String currentDriverController = DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort);
    String currentOperatorController = DriverStation.getJoystickName(ElectronicsIDs.OperatorControllerPort);
    Logger.recordOutput("Controllers/Driver", currentDriverController);
    Logger.recordOutput("Controllers/Operator", currentOperatorController);
    Logger.recordOutput("vision/differenceInPosition", Units
        .metersToFeet(Math.abs(robotContainer.driveSub.getPose().getX() - robotContainer.reefAutoTargetPose.getX())));
    Logger.recordOutput("vision/reefAutoTargetPose", robotContainer.reefAutoTargetPose);

    Logger.recordOutput("Arm/CurrentState", robotContainer.armState.getCurrentState());
    Logger.recordOutput("Arm/DesiredState", robotContainer.armState.getTargetState());

    Logger.recordOutput("Controllers/Driver/CurrentController", currentDriverController);
    Logger.recordOutput("Controllers/Operator/CurrentController", currentOperatorController);
    elevator.setLength(/* ArmConstants.ArmMinimumHeight + */(robotContainer.elevatorSub.getElevatorHeightInches()
        + robotContainer.elevatorSub.getCarriageHeightInches()) / 12.0);
    arm.setAngle(robotContainer.armSub.getArmTalonEncoderDegrees() - 90); // might need to change this to
                                                                          // getArmEncoderDegrees() instead, but (as of
                                                                          // right now) that doesn't work
    gripper.setAngle(robotContainer.wristSub.getWristEncoderDegrees());

    SmartDashboard.putData("ArmMech2D", mech);

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotContainer.disableSubsytems();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // if (!calibrationPerformed && Robot.isReal()) {
    // Command calibrateElevator = new
    // CalibrateElevator(robotContainer.elevatorSub);
    // Command calibrateCarriage = new
    // CalibrateCarriage(robotContainer.elevatorSub);

    // commandGroup.addCommands(calibrateElevator, calibrateCarriage, new
    // InstantCommand(() -> {this.calibrationPerformed = true;}));
    // }

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      commandGroup.addCommands(autonomousCommand.asProxy());
    }

    commandGroup.schedule();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    robotContainer.armSub.applyAllConfigs();

    SequentialCommandGroup calibrationSequence = new SequentialCommandGroup();

    // if (!calibrationPerformed && Robot.isReal()) {
    // Command calibrateElevator = new CalibrateElevator(robotContainer.armSub);
    // Command calibrateCarriage = new CalibrateCarriage(robotContainer.armSub);
    // Command setState = new InstantCommand(() ->
    // {robotContainer.armSub.setActualState(ArmState.Startup);
    // robotContainer.armSub.setDesiredState(ArmState.Startup);});

    // calibrationSequence.addCommands(calibrateElevator, calibrateCarriage, new
    // InstantCommand(() -> {this.calibrationPerformed = true;}));
    // calibrationSequence.schedule();
    // }

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}

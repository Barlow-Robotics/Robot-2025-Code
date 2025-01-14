// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechExtreme3DConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.DriveRobot;
import frc.robot.subsystems.Drive;

public class RobotContainer {
    private static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger moveToSpeakerButton; // button x
    private Trigger moveToAmpButton; // button y
    private Trigger moveToSourceButton; // left stick
    private Trigger moveToFloorButton; // left bumper
    private Trigger moveToFerryButton; // hamburger

    private Trigger climbButton; // button a
    private Trigger piviotToPoint;
    Drive driveSub;
  public RobotContainer() {
    driveSub = new Drive();
    configureBindings();
  }

  private void configureBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);
        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                // new DriveRobot(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true));

                // new DriveRobotWithAprilTagAlign(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true,
                // visionSub,
                // // shooterMountSub,
                // () -> autoAlignButton.getAsBoolean()));

                // new DriveRobotWithNoteAlign(
                        // driveSub,
                        // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                        // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                        // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                        // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                        // visionSub,
                        // floorIntakeSub,
                        // () -> autoAlignButton.getAsBoolean()));
                new DriveRobot(
                    driveSub,
                    () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                    () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                    () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                    () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                    false
                )
        );

        /***************** DRIVE *****************/

        // autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        // restartGyroButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        // restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

        /******************** SET SHOOTER MOUNT POSITION ********************/

        // moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
        // moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        // moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY);
        // moveToAmpButton.onTrue(setShooterPosAmpCmd);

        // moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.LeftStick);
        // moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        // moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
        // moveToFloorButton.onTrue(setShooterPosFloorCmd);

        // moveToFerryButton = new JoystickButton(operatorController, XboxControllerConstants.HamburgerButton);
        // moveToFerryButton.onTrue(setShooterPosFerryCmd);

        // /******************** SHOOTER & INTAKE ********************/

        // shootIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Trigger);
        // shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        // reverseFloorIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button7);
        // reverseFloorIntakeButton.onTrue(reverseFloorIntakeCmd).onFalse(stopShooterIntakeCmd);

        // /******************** CLIMB ********************/

        // climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA);
        // climbButton.onTrue(climbCmd);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

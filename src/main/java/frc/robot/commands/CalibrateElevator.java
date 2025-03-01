// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateElevator extends Command {

  private Arm armSub;

  public CalibrateElevator(Arm armSub) {
    this.armSub = armSub;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSub.setElevatorSpeed(ArmConstants.CalibrationVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setElevatorSpeed(ArmConstants.CalibrationVelocity);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.stopElevatorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(armSub.getElevatorCurrent()) > ArmConstants.CalibratonCurrentTolerance) && (Math.abs(armSub.getElevatorSpeed()) < ArmConstants.CalibratonVelocityTolerance);
 }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {

  private final Command setArmPosAlageEndCmd;
  private final Command runGripperCmd;
  private final Command setArmPosLoadCoralCmd;
  private Command currentCommand;
  
  public RemoveAlgae(Command setArmPosAlageEndCmd, Command runGripperCmd, Command setArmPosLoadCoralCmd) {
    this.setArmPosAlageEndCmd = setArmPosAlageEndCmd;
    this.runGripperCmd = runGripperCmd;
    this.setArmPosLoadCoralCmd = setArmPosLoadCoralCmd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentCommand = Commands.race(setArmPosAlageEndCmd, runGripperCmd).andThen(setArmPosLoadCoralCmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentCommand.isFinished();
  }
}

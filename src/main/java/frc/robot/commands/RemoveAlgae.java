// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae extends Command {

  private Command currentCommand;
  private final Arm armSub;
  private final Gripper gripperSub;
  

  public RemoveAlgae(Arm armSub, Gripper gripperSub) {

    this.armSub = armSub;
    this.gripperSub = gripperSub;
    addRequirements(armSub, gripperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmState currentState = armSub.getArmState();
    if (currentState == ArmState.StartAlgaePosition) {
      currentCommand = Commands.sequence(
          Commands.race(new SetArmPosition(armSub, ArmState.FinishRemovingAlgae), new RunGripper(gripperSub, armSub)),
          new SetArmPosition(armSub, ArmState.LoadCoral)
      );
    }  
    else{
      currentCommand = Commands.none();
    }
    currentCommand.schedule();
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

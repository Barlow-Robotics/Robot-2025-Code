// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Gripper.GripperState;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliverCoral extends Command {
  /** Creates a new DeliverCoral. */
  private boolean currentlyDoingAState;
  private boolean firstTime;
  private ArmState currentState; 
  private final Arm armSub;
  private final Gripper gripperSub;
  public DeliverCoral(Arm armSub, Gripper gripperSub) {
    this.gripperSub = gripperSub;
    this.armSub = armSub;
    addRequirements(armSub, gripperSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentlyDoingAState = false;
    firstTime = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentState = armSub.getArmState();
    if (!currentlyDoingAState && firstTime) {
      if (currentState == ArmState.ScoreLevel2 || currentState == ArmState.ScoreLevel3 || currentState == ArmState.ScoreLevel4) {
        gripperSub.setState(GripperState.placingCoral);
        armSub.setDesiredState(ArmState.WaitingForCoral);
        currentlyDoingAState = true;
        firstTime = false;
      }
      else if (currentState == ArmState.Level1) {
        gripperSub.setState(GripperState.releasingL1);
        currentlyDoingAState = true;
        firstTime = false;
    }

    if (currentState == ArmState.Level1 && gripperSub.getState() == GripperState.finishedReleasingL1) {
      armSub.setDesiredState(ArmState.SafeToLowerArm);
      gripperSub.setState(GripperState.placingCoral);
      currentlyDoingAState = true;
    }

    if (armSub.hasCompletedMovement()) {
      currentlyDoingAState = false;
    }
    if (currentState == ArmState.SafeToLowerArm && !firstTime && !currentlyDoingAState) {
      armSub.setDesiredState(ArmState.WaitingForCoral);
      currentlyDoingAState = true;
    }
      // Wait for it to complete. 

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


// Need to make a new motion magic for L1 
// Need to make the wrist turn later. 
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
public class ScoreCoral extends Command {

  private boolean currentlyMoving;
  private boolean firstTime;

  private ArmState currentState; 

  private final Arm armSub;
  private final Gripper gripperSub;

  public ScoreCoral(Arm armSub, Gripper gripperSub) {
    this.gripperSub = gripperSub;
    this.armSub = armSub;
    addRequirements(armSub, gripperSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentlyMoving = false;
    firstTime = true;
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentState = armSub.getArmState();
    if (!currentlyMoving && firstTime) {
      if (currentState == ArmState.Level2 || currentState == ArmState.Level3 || currentState == ArmState.Level4) {
        gripperSub.setState(GripperState.PlacingCoral);
        if (currentState  == ArmState.Level2)
          armSub.setDesiredState(ArmState.ScoreLevel2);
        if (currentState  == ArmState.Level3)
          armSub.setDesiredState(ArmState.ScoreLevel3);
        if (currentState  == ArmState.Level4)
          armSub.setDesiredState(ArmState.ScoreLevel4);


        currentlyMoving = true;
        firstTime = false;
      }
      else if (currentState == ArmState.Level1) {
        gripperSub.setState(GripperState.ReleasingL1);
        currentlyMoving = true;
        firstTime = false;
      }
    }

    if (currentState == ArmState.Level1 && gripperSub.getState() == GripperState.FinishedReleasingL1) {
      armSub.setDesiredState(ArmState.SafeToLowerArm);
      gripperSub.setState(GripperState.PlacingCoral);
      currentlyMoving = true;
    }

    if (armSub.hasCompletedMovement()) currentlyMoving = false;
    
    if (currentState == ArmState.SafeToLowerArm && !firstTime && !currentlyMoving) {
      // armSub.setDesiredState(ArmState.WaitingForCoral); // CHANGE: TO ADD LATER
      currentlyMoving = true;
    }
      // Wait for it to complete. 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == ArmState.SafeToLowerArm;
  }
}


// Need to make a new motion magic for L1 
// Need to make the wrist turn later. 
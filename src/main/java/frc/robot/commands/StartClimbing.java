// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Climb.ClimbState;

public class StartClimbing extends Command {
    /** Creates a new StartClimbing. */
    private final Climb climbSub;
    private final Arm armSub;
    private final ArmStateManager armStateManager ;

    public StartClimbing(Climb climbSub, Arm armSub, ArmStateManager armStateManager) {
        this.climbSub = climbSub;
        this.armSub = armSub;
        this.armStateManager = armStateManager;

        addRequirements(climbSub, armSub);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmState currentState = armStateManager.getCurrentState();
        if (currentState != ArmState.Running) {
            armStateManager.setTargetState(ArmState.Running);
        }
    }

    public void execute() {
        ArmState currentState = armStateManager.getCurrentState() ;
        if (currentState == ArmState.Running) {
            if (climbSub.getCurrentState() == ClimbState.Default) {
                climbSub.latchOntoCage();
            } else if (climbSub.getCurrentState() == ClimbState.WinchedOnCage && climbSub.returnButtonState() == 1) {
                climbSub.windWinch();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (climbSub.getCurrentState() == ClimbState.WinchedOnCage);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Climb.ClimbState;

public class DoClimb extends Command {
    /** Creates a new StartClimbing. */

    private Command comm;

    private final Climb climbSub;
    private final Arm armSub;
    private final ArmStateManager armStateManager;
    private final Elevator elevatorSub;
    private final Wrist wristSub;

    public DoClimb(Climb climbSub, Arm armSub, ArmStateManager armStateManager, Elevator elevatorSub, Wrist wristSub) {
        this.climbSub = climbSub;
        this.armSub = armSub;
        this.armStateManager = armStateManager;
        this.elevatorSub = elevatorSub;
        this.wristSub = wristSub;

        // comm = new SequentialCommandGroup();

        addRequirements(climbSub, armSub);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //comm = new SequentialCommandGroup();
        if (climbSub.getCurrentState() == ClimbState.Idle) {
            // comm = new InstantCommand(() -> climbSub.goToUnwind()) ;
            comm =         new PositionGripper(armStateManager, ArmState.Climb, elevatorSub, armSub, wristSub);
            // comm = Commands.sequence(
            //         new PositionGripper(armStateManager, ArmState.Climb, elevatorSub, armSub, wristSub),
            //         new InstantCommand(() -> climbSub.goToUnwind())
            //         );

        } else if (climbSub.getCurrentState() == ClimbState.ReadyToLatch) {
            comm =  new InstantCommand(() -> climbSub.goToWind());
        } else {
            comm = null ;
        }
        if ( comm != null) {
            comm.schedule();
        }
    }

    public void execute() {
        
        // if (climbSub.getCurrentState() == ClimbState.Idle) {
        //     comm = Commands.sequence(new InstantCommand(() -> climbSub.goToUnwind()));
        // } else if (climbSub.getCurrentState() == ClimbState.ReadyToLatch) {
        //     comm = Commands.sequence(new InstantCommand(() -> climbSub.goToWind()));
        // }

        // comm.schedule();
    }

    @Override
    public boolean isFinished() {
        if ( comm != null) {
            return (comm.isFinished());
        }
        return true ;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
// import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadCoralFromChute extends Command {

    Elevator theElevator ;
    Arm theArm;
    Wrist theWrist ;
    Gripper theGripper;
    ArmStateManager armState ;

    private Command currentCommand;

    /** Creates a new LoadCoralFromChute. */
    public LoadCoralFromChute(Elevator e, Arm a, Wrist w, Gripper g, ArmStateManager asm ) {
        theElevator = e ;
        theArm = a;
        theWrist = w ;
        theGripper = g;
        armState = asm ;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(theElevator, theArm, theWrist, theGripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        if ( armState.getCurrentState() != ArmState.WaitingForCoral ) {
            // what should we do? Ignore it?
            currentCommand = Commands.sequence(
                new PositionGripper(armState, ArmState.WaitingForCoral, theElevator, theArm, theWrist)
            ) ;

        } else {
            double startingCarriageHeight = theElevator.getCarriageHeightInches() ;
            currentCommand = Commands.sequence(
                // move arm to load position
                // start gripper spinng to intake
                // wait for gripper to have coral or a timeoout
                // return gripper to load position

                // wpk need to fix magic numbers
                new MoveElevator(theElevator, 0.0, startingCarriageHeight - 3.0) ,
                new InstantCommand( () -> theGripper.startIntaking() ),
                new MoveElevator(theElevator, 0.0, startingCarriageHeight) ,
                new WaitCommand(0.25) ,
                new StopGripper(theGripper) ,
                new PositionGripper(armState, ArmState.Running, theElevator, theArm, theWrist)
            ) ;

        }

        currentCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentCommand.isFinished();
    }
}

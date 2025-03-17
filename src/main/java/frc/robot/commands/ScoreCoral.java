// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Gripper.GripperState;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {

    private ArmStateManager armStateManager ;
    private ArmState currentState;

    private final Elevator theElevator ;
    private final Arm theArm;
    private final Wrist theWrist ;
    private final Gripper theGripper ;

    private Command currentCommand;


    // private final Gripper gripperSub;

    public ScoreCoral(ArmStateManager asm, Elevator e, Arm a, Wrist w,  Gripper g) {
        armStateManager = asm ;
        theElevator = e ;
        theArm = a ;
        theWrist = w ;
        theGripper = g ;

        addRequirements(theElevator, theArm, theWrist, theGripper );
    }


    private Command createL234ScoreCommand( double deltaElevatorHeight, double deltaCarriageHeight, double deltaArmAngle) {

        return Commands.sequence(
            // new InstantCommand(() -> theGripper.setReleaseMode() ),
            new ParallelCommandGroup(
                new InstantCommand( ()-> theGripper.releaseCoral()),
                // new MoveArm( theArm, theArm.getArmEncoderDegrees()+deltaArmAngle) ,
                new MoveElevator(theElevator, theElevator.getDesiredElevatorHeightInches()+deltaElevatorHeight, theElevator.getDesiredCarriageHeightInches()+deltaCarriageHeight) 
            ) ,
            // new MoveElevator(theElevator, theElevator.getDesiredElevatorHeightInches()+deltaArmAngle, theElevator.getDesiredCarriageHeightInches()+deltaArmAngle-1.0), 
            new InstantCommand( ()-> theGripper.stop()),
            new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist)
            );            
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        currentState = armStateManager.getCurrentState();

        if (currentState == ArmState.Level1) {
            currentCommand = Commands.sequence(
                new EjectCoral( theGripper).withTimeout(1.0) ,
                new StopGripper(theGripper),
                new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist) 
            ) ;
        } else if (currentState == ArmState.Level2) {
            currentCommand = createL234ScoreCommand(0, -7.0, -30) ;
        } else if (currentState == ArmState.Level3) {
            currentCommand = createL234ScoreCommand(0, -7.0, -30) ;
        } else if (currentState == ArmState.Level4){
            currentCommand = createL234ScoreCommand(0, -4.0, -30) ;
        } else {
            // can't score coral if your not at one of the levels
            currentCommand = new NoOpCommand() ;
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
        return currentCommand.isFinished() ;
    }
}


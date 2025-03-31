// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Set;

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
public class ScoreCoral {

    private ArmStateManager armStateManager;
    private ArmState currentState;

    private final Elevator theElevator;
    private final Arm theArm;
    private final Wrist theWrist;
    private final Gripper theGripper;

    private boolean goToTravel = true ;

    public ScoreCoral(ArmStateManager asm, Elevator e, Arm a, Wrist w, Gripper g) {
        armStateManager = asm;
        theElevator = e;
        theArm = a;
        theWrist = w;
        theGripper = g;
    }

    public ScoreCoral withGoToTravel( boolean value) {
        goToTravel = value ;
        return this ;
    }

    public Command command() {
        return Commands.defer(() -> {
            currentState = armStateManager.getCurrentState();

            if (currentState == ArmState.Level1) {
                return createL1ScoreCommand();
            } else if (currentState == ArmState.Level2) {
                return createL234ScoreCommand(0, -7.0, -30);
            } else if (currentState == ArmState.Level3) {
                return createL234ScoreCommand(0, -7.0, -30);
            } else if (currentState == ArmState.Level4) {
                return createL234ScoreCommand(0, -4.0, -30);
            } else {
                // can't score coral if your not at one of the levels
                return Commands.none();
            }
        },
                Set.of(theElevator, theArm, theWrist, theGripper));

    }


    private Command createL1ScoreCommand() {
        ArrayList<Command> commands = new ArrayList<Command>() ;
        commands.add(new EjectCoral(theGripper).withTimeout(1.0) ) ;
        commands.add(new StopGripper(theGripper) ) ;
        if ( goToTravel ) {
            commands.add(new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist).command() ) ;
        }
        return Commands.sequence( commands.toArray(new Command[0])) ;
    }



    private Command createL234ScoreCommand(double deltaElevatorHeight, double deltaCarriageHeight,
            double deltaArmAngle) {

        ArrayList<Command> commands = new ArrayList<Command>();

        // commands.add(new InstantCommand(() -> theGripper.setReleaseMode() ) ) ;
        commands.add(new ParallelCommandGroup(
                new InstantCommand(() -> theGripper.releaseCoral()),
                // new MoveArm( theArm, theArm.getArmEncoderDegrees()+deltaArmAngle) ,
                new MoveElevator(theElevator,
                        theElevator.getDesiredElevatorHeightInches() + deltaElevatorHeight,
                        theElevator.getDesiredCarriageHeightInches() + deltaCarriageHeight)));
        commands.add(new InstantCommand(() -> theGripper.stop()));

        if (goToTravel) {
            commands.add(
                    new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist).command());
        }

        return Commands.sequence(commands.toArray(new Command[0]));
    }

    private Command createOptimizedL234ScoreCommand(double deltaElevatorHeight, double deltaCarriageHeight,
            double deltaArmAngle) {

        ArrayList<Command> commands = new ArrayList<Command>();

        commands.add(new ParallelCommandGroup(
                new InstantCommand(() -> theGripper.releaseCoral()),
                // new MoveArm( theArm, theArm.getArmEncoderDegrees()+deltaArmAngle) ,
                new MoveElevator(theElevator,
                        theElevator.getDesiredElevatorHeightInches() + deltaElevatorHeight,
                        theElevator.getDesiredCarriageHeightInches() + deltaCarriageHeight)));
        commands.add(new InstantCommand(() -> theGripper.stop()));

        if (goToTravel) {
            commands.add(
                    new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist).command());
        }

        return Commands.sequence(commands.toArray(new Command[0]));
    }




}

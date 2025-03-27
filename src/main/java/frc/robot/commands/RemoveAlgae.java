// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RemoveAlgae {

    private final ArmStateManager armStateManager;
    private final Elevator theElevator;
    private final Arm theArm;
    private final Wrist theWrist;
    private final Gripper theGripper;

    public RemoveAlgae(ArmStateManager asm, Elevator e, Arm a, Wrist w, Gripper g) {
        armStateManager = asm;
        theElevator = e;
        theArm = a;
        theWrist = w;
        theGripper = g;
    }

    public Command command() {
        return Commands.defer(
                () -> {
                    ArmState currentState = armStateManager.getCurrentState();
                    if (currentState != ArmState.StartAlgaePosition) {
                        // we need to position the gripper for the next button press
                        return Commands.sequence(
                                new PositionGripper(armStateManager, ArmState.StartAlgaePosition, theElevator, theArm,
                                        theWrist)
                                        .command());
                    } else {
                        return Commands.sequence(
                                // wpk need to fix magic numbers
                                new InstantCommand(() -> theGripper.startAlgaeRemoval()),
                                // move the elevator up to strip the algae
                                // new MoveElevator(theElevator,
                                //         theElevator.getDesiredElevatorHeightInches() + 8.0,
                                //         ArmConstants.ElevatorAlgaeRemovalVelocity,
                                //         theElevator.getDesiredCarriageHeightInches() + 0.0,
                                //         ArmConstants.CarriageAlgaeRemovalVelocity),
                                new MoveElevator(theElevator,
                                        theElevator.getDesiredElevatorHeightInches() + 7.5,
                                        ArmConstants.ElevatorAlgaeRemovalVelocity,
                                        20.0,
                                        ArmConstants.CarriageAlgaeRemovalVelocity),
                                new InstantCommand(() -> theGripper.stop()),
                                new PositionGripper(armStateManager, ArmState.Running, theElevator, theArm, theWrist)
                                        .command());
                    }

                },
                Set.of(theElevator, theArm, theWrist, theGripper));

    }
}

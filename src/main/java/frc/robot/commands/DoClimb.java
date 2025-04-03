// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Climb.ClimbState;

public class DoClimb {
    /** Creates a new StartClimbing. */

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
    }

    public Command command() {
        return Commands.defer(
                () -> {
                    if (climbSub.getCurrentState() == ClimbState.Idle) {
                        return Commands.sequence(                      
                        // new InstantCommand(() -> climbSub.goToUnwind()),
                        new PositionGripper(armStateManager, ArmState.Climb, elevatorSub, armSub, wristSub).command(),                       
                        Commands.none()  // wpk this is a hack to work around command not finishing
                        );

                    } else if (climbSub.getCurrentState() == ClimbState.ReadyToLatch) {
                        return new InstantCommand(() -> climbSub.goToWind());
                    } else if ( climbSub.getCurrentState() == ClimbState.Holding) {
                        return new PositionGripper(armStateManager, ArmState.Running, elevatorSub, armSub, wristSub).command();                       
                    } else {
                        return Commands.none();
                    }
                },
                Set.of(climbSub, armSub, elevatorSub, wristSub));
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadCoralFromChute extends Command {

    Arm armSub;
    Gripper gripper;

    private Command currentCommand;

    /** Creates a new LoadCoralFromChute. */
    public LoadCoralFromChute(Arm a, Gripper g) {
        armSub = a;
        gripper = g;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSub, gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ArmState currentState = armSub.getArmState();
        if (currentState != ArmState.WaitingForCoral ) {
            currentCommand = new SetArmPosition(armSub, ArmState.WaitingForCoral) ;
                    
        } else {
            currentCommand = Commands.sequence(
                new RunGripperToRemoveAlgae(gripper) ,
                new SetArmPosition(armSub, ArmState.LoadCoral),
                new SetArmPosition(armSub, ArmState.PostLoadCoral),
                new StopGripper(gripper));
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
        return false;
    }
}

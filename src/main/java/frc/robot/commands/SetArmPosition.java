// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class SetArmPosition extends Command {

    protected Arm armSub;
    protected ArmState desiredState;

    public SetArmPosition(Arm armSub, ArmState desiredState) {
        this.armSub = armSub;
        this.desiredState = desiredState;
        addRequirements(armSub);
    }

    @Override
    public void initialize() {
        armSub.setDesiredState(desiredState);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override                                                                                                                                                                
    public boolean isFinished() {
        return armSub.hasCompletedMovement() ;
    }   

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateCarriage extends Command {

    private Elevator theElevator;

    public CalibrateCarriage(Elevator e) {
        theElevator = e;
        addRequirements(theElevator);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        theElevator.setCarriageSpeed(ArmConstants.CalibrationVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        theElevator.setCarriageSpeed(ArmConstants.CalibrationVelocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        theElevator.stopCarriageMotor();
        theElevator.resetCarriageEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(theElevator.getCarriageCurrent()) > ArmConstants.CalibratonCurrentTolerance)
                && (Math.abs(theElevator.getCarriageSpeed()) < ArmConstants.CalibratonVelocityTolerance);
    }
}

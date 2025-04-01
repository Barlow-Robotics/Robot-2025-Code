// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForArmMovement extends Command {

    Arm armSub ;
    double angleDelta = 0.0 ;
    double startingAngle = 0.0 ;


    /** Creates a new WaitForArmMovement. */
    public WaitForArmMovement(Arm a, double change) {
        armSub = a ;
        angleDelta = change ;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingAngle = armSub.getArmEncoderDegrees() ;
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
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if we're waiting for the elevator to go down
        if ( angleDelta < 0.0) {
            if ( armSub.getArmEncoderDegrees() < (startingAngle+angleDelta)) {
                int wpk = 1 ;
            }
            return armSub.getArmEncoderDegrees() < (startingAngle+angleDelta) ;
        } else {
            if ( armSub.getArmEncoderDegrees() >= (startingAngle+angleDelta)) {
                int wpk = 1 ;
            }

            return armSub.getArmEncoderDegrees() > (startingAngle+angleDelta) ;
        }
    }
}

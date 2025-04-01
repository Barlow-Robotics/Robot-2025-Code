// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WaitForElevatorToTravel extends Command {

    Elevator elevator;
    double heightDelta = 0.0 ;
    double startingHeight = 0.0 ;

    /** Creates a new WaitForElevatorToTravel. */
    public WaitForElevatorToTravel(Elevator e, double change) {
        // Use addRequirements() here to declare subsystem dependencies.
        elevator = e ;
        heightDelta = change ;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingHeight = elevator.getTotalHeightInches() ;
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
        // if we're waiting for the elevator to go down
        if ( heightDelta < 0.0) {
            if (elevator.getTotalHeightInches() < (startingHeight+heightDelta)) {
                int wpk = 1 ;
            }
            return elevator.getTotalHeightInches() < (startingHeight+heightDelta) ;
        } else {
            return elevator.getTotalHeightInches() > (startingHeight+heightDelta) ;
        }
    }
}

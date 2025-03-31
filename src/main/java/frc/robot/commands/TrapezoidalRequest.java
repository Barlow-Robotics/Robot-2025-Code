// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrapezoidalRequest extends Command {

    Drive driveSub;
    Pose2d targetPose;


    private final ProfiledPIDController displacementPID ;
    private final ProfiledPIDController rotationPID;

    /** Creates a new TrapazoidalRequest. */
    public TrapezoidalRequest(Drive driveSub, Pose2d targetPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveSub = driveSub;
        this.targetPose = targetPose;

        displacementPID = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(2.0, 2.0));

        rotationPID = new ProfiledPIDController(1.0, 0, 0, new TrapezoidProfile.Constraints(2.0*Math.PI, 8.0* Math.PI));
        rotationPID.setTolerance(Units.degreesToRadians(2.0)) ;
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        displacementPID.setTolerance(0.03);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // set goals in profiled PIDs
        Translation2d translationDelta = driveSub.getPose().getTranslation().minus(targetPose.getTranslation());
        double displacement = translationDelta.getNorm() ;
        double rotationDelta = driveSub.getPose().getRotation().minus(targetPose.getRotation()).getRadians();

        // displacementPID.reset( displacement) ;
        Translation2d speedVector = new Translation2d( driveSub.getState().Speeds.vxMetersPerSecond, driveSub.getState().Speeds.vyMetersPerSecond) ;
        displacementPID.reset( new State(displacement, speedVector.getNorm())) ;

        rotationPID.reset(  rotationDelta );
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // set velocities based on PID calculations and feed forward values
        Translation2d translationDelta = driveSub.getPose().getTranslation().minus(targetPose.getTranslation());
        double displacement = translationDelta.getNorm() ;
        double rotationDelta = driveSub.getPose().getRotation().minus(targetPose.getRotation()).getRadians();

        double displacementPIDValue = displacementPID.calculate(displacement) ;
        
        //Get the unit vector of the translation delta and multply by the desired velocity and PID correction values
        Translation2d velocityVector = translationDelta.div(displacement).times(displacementPID.getSetpoint().velocity) ;
        Translation2d displacementPIDCorrection = translationDelta.div(displacement).times(displacementPIDValue) ;

        double rotPIDInput = rotationPID.calculate(  rotationDelta, 0);

        // Always use BlueAlliance for the ForwardPerspective value. This is because
        // all of our poses are always relative to blue-alliance.
        FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(displacementPIDCorrection.getX()+velocityVector.getX())
                .withVelocityY(displacementPIDCorrection.getY()+velocityVector.getY() )
                .withRotationalRate(rotPIDInput + rotationPID.getSetpoint().velocity );

        driveSub.setControl(swerveRequest);

        Logger.recordOutput("AutoAlign/poseX", driveSub.getPose().getTranslation().getX());
        Logger.recordOutput("AutoAlign/targetX", targetPose.getTranslation().getX());
        Logger.recordOutput("AutoAlign/poseX", driveSub.getPose().getTranslation().getX());

        Logger.recordOutput("AutoAlign/translationDeltaX", translationDelta.getX());
        Logger.recordOutput("AutoAlign/translationDeltaY", translationDelta.getY());
        Logger.recordOutput("AutoAlign/displacement", displacement);
        Logger.recordOutput("AutoAlign/rotationDelta", rotationDelta);

        Logger.recordOutput("AutoAlign/pidRotVelocity", rotationPID.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/requestXVelocity", swerveRequest.VelocityX);
        Logger.recordOutput("AutoAlign/requestYVelocity", swerveRequest.VelocityY);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Logger.recordOutput("Auto/displacementPID/atGoal", displacementPID.atGoal());
        Logger.recordOutput("Auto/rotationPID/atGoal", rotationPID.atGoal());

        return displacementPID.atGoal() && rotationPID.atGoal();
    }
}

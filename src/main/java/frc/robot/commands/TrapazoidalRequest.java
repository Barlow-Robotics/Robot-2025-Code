// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrapazoidalRequest extends Command {

    Drive driveSub;
    Pose2d targetPose;

    private final ProfiledPIDController profiledPIDX;
    private final ProfiledPIDController profiledPIDY;
    private final ProfiledPIDController profiledPIDRot;

    private final SimpleMotorFeedforward feedForwardX;
    private final SimpleMotorFeedforward feedForwardY;
    private final SimpleMotorFeedforward feedForwardRot;

    private final ProfiledPIDController profiledPIDTest;


    /** Creates a new TrapazoidalRequest. */
    public TrapazoidalRequest(Drive driveSub, Pose2d targetPose) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driveSub = driveSub;
        this.targetPose = targetPose;

        profiledPIDX = new ProfiledPIDController(8, 0, 0, new TrapezoidProfile.Constraints(3.0, 12.0));
        profiledPIDY = new ProfiledPIDController(8, 0, 0, new TrapezoidProfile.Constraints(3.0, 12.0));
        profiledPIDRot = new ProfiledPIDController(2.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI * 4.0));
        profiledPIDRot.setTolerance(Units.degreesToRadians(2.0)) ;
        profiledPIDRot.enableContinuousInput(-Math.PI, Math.PI);

        profiledPIDTest = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(3.0, 12.0));


        feedForwardX = new SimpleMotorFeedforward(0, 1.0 / Constants.VisionConstants.AutoAlignVelocityConstant);
        feedForwardY = new SimpleMotorFeedforward(0, 1.0 / Constants.VisionConstants.AutoAlignVelocityConstant);
        feedForwardRot = new SimpleMotorFeedforward(0,
                1.0 / Constants.VisionConstants.AutoAlignAngularVelocityConstant);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // set goals in profiled PIDs
        // profiledPIDX.setGoal(targetPose.getX());
        // profiledPIDY.setGoal(targetPose.getY());
        // profiledPIDRot.setGoal(targetPose.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // set velocities based on PID calculations and feed forward values
        // Translation2d translationDelta = (targetPose.getTranslation()).minus(driveSub.getPose().getTranslation());
        Translation2d translationDelta = driveSub.getPose().getTranslation().minus(targetPose.getTranslation());
        double rotationDelta = targetPose.getRotation().minus(driveSub.getPose().getRotation()).getRadians();

        var xPIDInput = profiledPIDX.calculate(driveSub.getPose().getTranslation().getX(), targetPose.getX());
        var yPIDInput = profiledPIDY.calculate(driveSub.getPose().getTranslation().getY(), targetPose.getY());
        var rotPIDInput = profiledPIDRot.calculate(  driveSub.getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

        var testPID = profiledPIDTest.calculate( driveSub.getPose().getTranslation().getX(), targetPose.getTranslation().getX()) ;


        // // Always use BlueAlliance for the ForwardPerspective value. This is because
        // // all of our poses are always relative to blue-alliance.
        // FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
        //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        //         .withVelocityX(xPIDInput + profiledPIDX.getSetpoint().velocity)
        //         .withVelocityY(yPIDInput + profiledPIDY.getSetpoint().velocity)
        //         .withRotationalRate(rotPIDInput + profiledPIDRot.getSetpoint().velocity);

        // Always use BlueAlliance for the ForwardPerspective value. This is because
        // all of our poses are always relative to blue-alliance.
        FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withVelocityX(xPIDInput)
                .withVelocityY(yPIDInput )
                .withRotationalRate(rotPIDInput );

        // FieldCentric swerveRequest = new SwerveRequest.FieldCentric()
        //         .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        //         .withVelocityX(profiledPIDX.getSetpoint().velocity)
        //         .withVelocityY(profiledPIDY.getSetpoint().velocity)
        //         .withRotationalRate(profiledPIDRot.getSetpoint().velocity);


        Logger.recordOutput("AutoAlign/poseX", driveSub.getPose().getTranslation().getX());
        Logger.recordOutput("AutoAlign/targetX", targetPose.getTranslation().getX());

        Logger.recordOutput("AutoAlign/translationDeltaX", translationDelta.getX());
        Logger.recordOutput("AutoAlign/translationDeltaY", translationDelta.getY());
        Logger.recordOutput("AutoAlign/rotationDelta", rotationDelta);
        Logger.recordOutput("AutoAlign/testVelocity", profiledPIDTest.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/testPID", testPID);
        Logger.recordOutput("AutoAlign/testPIDError", targetPose.getTranslation().getX()-driveSub.getPose().getTranslation().getX() );

        Logger.recordOutput("AutoAlign/pidX", xPIDInput);
        Logger.recordOutput("AutoAlign/pidY", yPIDInput);
        Logger.recordOutput("AutoAlign/pidRot", rotPIDInput);

        Logger.recordOutput("AutoAlign/pidXVelocity", profiledPIDX.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/pidYVelocity", profiledPIDY.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/pidRotVelocity", profiledPIDRot.getSetpoint().velocity);

        // Logger.recordOutput("AutoAlign/feedForwardX", xFeedForward);
        // Logger.recordOutput("AutoAlign/feedForwardY", yFeedForward);
        // Logger.recordOutput("AutoAlign/feedForwardRot", rotFeedForward);

        // Logger.recordOutput("AutoAlign/VelocityX", swerveRequest.VelocityX);
        // Logger.recordOutput("AutoAlign/VelocityY", swerveRequest.VelocityY);
        // Logger.recordOutput("AutoAlign/RotationalRate", swerveRequest.RotationalRate);

        driveSub.setControl(swerveRequest);

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

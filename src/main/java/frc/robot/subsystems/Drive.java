// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import org.json.simple.parser.ParseException;

import org.littletonrobotics.junction.Logger;
import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;

public class Drive extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            "frontLeft",
            ElectronicsIDs.FrontLeftDriveMotorID,
            ElectronicsIDs.FrontLeftTurnMotorID,
            ElectronicsIDs.FrontLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontLeftMagnetOffsetInRadians) / 360.0,
            false);

    private final SwerveModule frontRight = new SwerveModule(
            "frontRight",
            ElectronicsIDs.FrontRightDriveMotorID,
            ElectronicsIDs.FrontRightTurnMotorID,
            ElectronicsIDs.FrontRightTurnEncoderID,
            Math.toDegrees(DriveConstants.FrontRightMagnetOffsetInRadians) / 360.0,
            true);

    private final SwerveModule backLeft = new SwerveModule(
            "backLeft",
            ElectronicsIDs.BackLeftDriveMotorID,
            ElectronicsIDs.BackLeftTurnMotorID,
            ElectronicsIDs.BackLeftTurnEncoderID,
            Math.toDegrees(DriveConstants.BackLeftMagnetOffsetInRadians) / 360.0,
            false);

    private final SwerveModule backRight = new SwerveModule(
            "backRight",
            ElectronicsIDs.BackRightDriveMotorID,
            ElectronicsIDs.BackRightTurnMotorID,
            ElectronicsIDs.BackRightTurnEncoderID,
            Math.toDegrees(DriveConstants.BackRightMagnetOffsetInRadians) / 360.0,
            true);

    private AHRS navX; 

    private final SwerveDriveOdometry odometry;

    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[4];

//     private final Vision visionSub;

    private Pose2d currPose2d;
    private Pose2d lastPose2d;

    // private final AprilTagFieldLayout aprilTagFieldLayout;

    public Drive() {
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI, 100);

        // navX = new AHRS(Port.kMXP);
        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // zeroHeading();
        // } catch (Exception e) {
        // }
        // }).start();

        // this.visionSub = visionSub;

        odometry = new SwerveDriveOdometry(
                DriveConstants.kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        // wpk The pose estimator functionality was addapted from the
        // SwerveDrivePoseEstimator example provided
        // with WPILib
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition() },
                new Pose2d() ,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                // VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        // visionSub.photonEstimator.setReferencePose(this.getPose());

        // AprilTagFieldLayout layout;
        // try {
        //     layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        //     var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //         layout.setOrigin(
        //                 DriverStation.getAlliance().get() == Alliance.Blue
        //                         ? OriginPosition.kBlueAllianceWallRightSide
        //                         : OriginPosition.kRedAllianceWallRightSide);
        //     } else {
        //         // default this for now
        //         layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        //     }

        // } catch (IOException e) {
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        //     layout = null;
        // }
        // this.aprilTagFieldLayout = layout;
        lastPose2d = this.getPose();

    }

    @Override
    public void periodic() {
        odometry.update(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        poseEstimator.update(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        // var photonEstimate = visionSub.getEstimatedGlobalPose();

        // Vision estimate is not stable. Just use pure odometry for now
        // Don't re-enable until we have done more thorough testing of the
        // vision calibration and initialization.
        
        // if (photonEstimate.isPresent()) {
        //     poseEstimator.addVisionMeasurement(photonEstimate.get().estimatedPose.toPose2d(),
        //             photonEstimate.get().timestampSeconds);
        // }
        

        Logger.recordOutput("Drive/PoseEstimate", poseEstimator.getEstimatedPosition());
        // if (photonEstimate.isPresent()) {
        //     Logger.recordOutput("Drive/PhotonPoseEstimate", photonEstimate.get().estimatedPose.toPose2d());
        // }

        SwerveModuleState[] swerveModuleActualStates = new SwerveModuleState[] { frontLeft.getState(),
                frontRight.getState(), backLeft.getState(), backRight.getState() };
        logData(swerveModuleActualStates);
        if (currPose2d != null) {
                lastPose2d = currPose2d;
        }
        currPose2d = this.getPose();
        
    }

    private void logData(SwerveModuleState[] swerveModuleActualStates) {
        Logger.recordOutput("Drive/StatesActual", swerveModuleActualStates);
        Logger.recordOutput("Drive/Pose", getPose());
        Logger.recordOutput("Drive/PoseEstimate", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Heading", getHeading());
        Logger.recordOutput("Drive/Odometry/X", odometry.getPoseMeters().getX());
        Logger.recordOutput("Drive/Odometry/Y", odometry.getPoseMeters().getY());
        Logger.recordOutput("Drive/CurrentSupply/FrontLeftDrive", frontLeft.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontLeftTurn", frontLeft.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontRightDrive", frontRight.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/FrontRightTurn", frontRight.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackLeftDrive", backLeft.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackLeftTurn", backLeft.getTurnCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackRightDrive", backRight.getDriveCurrent());
        Logger.recordOutput("Drive/CurrentSupply/BackRightTurn", backRight.getTurnCurrent());
    }

    // // wpk consider deleting after pose estimation is tested.
    // public Pose2d getPoseWithoutVision() {
    //     return odometry.getPoseMeters();
    // }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        // return poseEstimator.getEstimatedPosition();
    }

    // public Pose2d getPoseWithVision() {
    // return poseEstimator.getEstimatedPosition();
    // }

    public void resetOdometry(Pose2d pose) {
        // Reset the angle adjustment to 0 so it doesn't influence the returned values
        navX.setAngleAdjustment(0);

        // Use the incoming pose to find the blueTargetHeading. This is the angle the robot should be at
        // relatice to a blue-origin.
        double targetHeading = pose.getRotation().getDegrees();

        // If we are the red alliance, we want field-relative drive to be rotated 180 degrees, so
        // adgjust the target heading accordingly
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                targetHeading += 180;
        }            

        // Find the real currentHeading based on navX.
        double currentHeading = navX.getRotation2d().getDegrees();

        // Set an angle adustment so that navX will report that we currently have
        // the target heading.
        //
        // NOTE: This is the negative of what you would expect because navX ANGLES
        // are inverted relatice to pose rotatoin angles.
        navX.setAngleAdjustment(currentHeading - targetHeading);

        Logger.recordOutput("Drive/Auto/CurrentHeading", currentHeading);
        Logger.recordOutput("Drive/Auto/TargetHeading", targetHeading);
        Logger.recordOutput("Drive/Auto/AngleAdjustment", currentHeading - targetHeading);

        odometry.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);

        poseEstimator.resetPosition(
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }

    // public void drive(double xSpeed, double ySpeed, double rot, boolean
    // fieldRelative) {
    // var swerveModuleDesiredStates =
    // DriveConstants.kinematics.toSwerveModuleStates(
    // fieldRelative
    // ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
    // navX.getRotation2d())
    // : new ChassisSpeeds(xSpeed, ySpeed, rot));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates,
    // DriveConstants.MaxDriveableVelocity);
    // frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
    // frontRight.setDesiredState(swerveModuleDesiredStates[1]);
    // backLeft.setDesiredState(swerveModuleDesiredStates[2]);
    // backRight.setDesiredState(swerveModuleDesiredStates[3]);

    // Logger.recordOutput("Drive/StatesDesired", swerveModuleDesiredStates);

    // }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleDesiredStates = DriveConstants.kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                                navX.getRotation2d())
                        : ChassisSpeeds.discretize(
                                new ChassisSpeeds(xSpeed, ySpeed, rot),
                                DriveConstants.TimestepDurationInSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleDesiredStates,
                DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(swerveModuleDesiredStates[0]);
        frontRight.setDesiredState(swerveModuleDesiredStates[1]);
        backLeft.setDesiredState(swerveModuleDesiredStates[2]);
        backRight.setDesiredState(swerveModuleDesiredStates[3]);

        Logger.recordOutput("Drive/StatesDesired", swerveModuleDesiredStates);
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
                DriveConstants.TimestepDurationInSeconds);
        SwerveModuleState[] targetStates = DriveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates,
                Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.DriveConstants.MaxDriveableVelocity);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void zeroHeading() {
        navX.setAngleAdjustment(0);
        navX.reset();
    }

    public double getHeading() {
        return navX.getRotation2d().getDegrees();
    }

    public double getTurnDegPSec() {
        return navX.getRate() * (DriveConstants.GyroReversed ? -1.0 : 1.0); // degrees per second
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /* SIMULATION */

    public void simulationInit() {
        frontLeft.simulationInit();
        frontRight.simulationInit();
        backLeft.simulationInit();
        backRight.simulationInit();

        previousPositions[0] = frontLeft.getPosition();
        previousPositions[1] = frontRight.getPosition();
        previousPositions[2] = backLeft.getPosition();
        previousPositions[3] = backRight.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        var modulePositions = new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };

        var moduleDeltas = new SwerveModulePosition[modulePositions.length];
        for (int index = 0; index < modulePositions.length; index++) {
            var current = modulePositions[index];
            if (previousPositions[0] == null) {
                previousPositions[0] = frontLeft.getPosition();
                previousPositions[1] = frontRight.getPosition();
                previousPositions[2] = backLeft.getPosition();
                previousPositions[3] = backRight.getPosition();   
            }
        //     System.out.println(previousPositions[0]);
            var previous = previousPositions[index];

            moduleDeltas[index] = new SwerveModulePosition(current.distanceMeters - previous.distanceMeters,
                    current.angle);
            previous.distanceMeters = current.distanceMeters;
        }
        var twist = DriveConstants.kinematics.toTwist2d(moduleDeltas);

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(navX.getAngle() - navX.getAngleAdjustment() - Units.radiansToDegrees(twist.dtheta));
    }

    public Command ChoreoAuto(String name) {
        try {
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
                return AutoBuilder.followPath(path).alongWith(Commands.runOnce(() -> resetOdometry(path.getStartingDifferentialPose())));
                // Do something with the path
        } catch (IOException e) {
                e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
        } catch (ParseException e) {
                e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
        }
        return Commands.none();
        }
    public Command ChoreoAutoWithoutReset(String name) {
        PathPlannerPath path;
        try {
                path = PathPlannerPath.fromChoreoTrajectory(name);
                // Do something with the path
        } catch (IOException e) {
                e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
                path = null;
        } catch (ParseException e) {
                e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
                path = null;
        }
        return AutoBuilder.followPath(path);
    }

    public boolean isMoving() {
        return lastPose2d.equals(currPose2d);
    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.TuneableParameter;

public class Constants {

    public static final double SecondsPerMinute = 60;
    public static final double jKgMetersSquared = 0.0005;
    public static final double Neo550MaxRPM = 11000;
    public static final double NeoMaxRPM = 5820;
    public static final double Falcon500MaxRPM = 6300;
    public static final double KrakenX60MaxRPM = 6000;

    public static final double TalonFXResolution = 2048;
    public static final double CANcoderResolution = 4096;
    
    public static final boolean IsFocEnabled = true; //applies to all TalonFXs (except drive)

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/
    public static final class VisionConstants {
        public static final int CameraLightID = 0; // NEED TO FIX/CHANGE
        public static final String ClimbCameraName = "Climb_Camera";
        public static final String ElevatorCameraName = "Reef_Camera";
        public static final String RightClimbCamName = "";

        public static final PoseStrategy PrimaryVisionStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final PoseStrategy FallbackVisionStrategy = PoseStrategy.LOWEST_AMBIGUITY;

        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        // NEED TO FIX: wpk need to update these to be more exact.
        public static final Transform3d ClimbCameraToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(DriveConstants.TotalWidth/2-12.625), Units.inchesToMeters(-1.0*(DriveConstants.TotalWidth / 2)+2.5), Units.inchesToMeters(13.125)),
                new Rotation3d(0, Units.degreesToRadians(0), 0));
        public static final Transform3d RobotToClimbCamera = ClimbCameraToRobot.inverse();

        public static final Transform3d ElevatorCamToRobot = 
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(DriveConstants.TotalWidth/2-2.5), Units.inchesToMeters(DriveConstants.TotalWidth/2-9.25), Units.inchesToMeters(12.625)), 
                    new Rotation3d(0, Units.degreesToRadians(0), 0));
        public static final Transform3d RobotToElevatorCam = ElevatorCamToRobot.inverse();

        public static final Transform3d RightClimbCamToRobot = 
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(DriveConstants.TotalWidth/2-2.5), Units.inchesToMeters(DriveConstants.TotalWidth/2-9.25), Units.inchesToMeters(12.625)), 
                    new Rotation3d(0, Units.degreesToRadians(0), 0)); //  NEED TO FIX. 
        public static final Transform3d RobotToRightClimbCam = RightClimbCamToRobot.inverse();


        

        // // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout FieldTagLayout = AprilTagFields.k2025ReefscapeAndyMark
                .loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(1, 1, 0.1);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        // constants for vision-calculated speaker shooting - LMT
        public static final int NullAprilTagID = -1;
        public static final double InvalidAngle = -361;
        public static final double NoTargetDistance = -1;

        public static final double AutoAlignVelocityConstant = 3; 

        public static final int[] blueAprilTagListReef = {17, 18, 19, 20, 21, 22};
        public static final int[] redAprilTagListReef = {6, 7, 8, 9, 10, 11};


        public static final int[] blueAprilTagListCoralStation = {12, 13};
        public static final int[] redAprilTagListCoralStation = {1, 2};

    
        public static final double NoteAlignPixelTolerance = 250; // NEED TO CHANGE

    }

    public static final class ElectronicsIDs {

        public static final int DriverControllerPort = 1;
        public static final int OperatorControllerPort = 2;
        public static final int TestControllerPort = 0;

        /***************************** DRIVE *****************************/
        // CANcoder = 1{locationOnBot}
        public static final int FrontLeftTurnEncoderID = 11;
        public static final int FrontRightTurnEncoderID = 12;
        public static final int BackLeftTurnEncoderID = 13;
        public static final int BackRightTurnEncoderID = 14;

        // DriveMotorID = 2{locationOnBot} // Base
        public static final int FrontLeftDriveMotorID = 21;
        public static final int FrontRightDriveMotorID = 22;
        public static final int BackLeftDriveMotorID = 23;
        public static final int BackRightDriveMotorID = 24;

        // TurnMotorID = 3{locationOnBot} // Side
        public static final int FrontLeftTurnMotorID = 31;
        public static final int FrontRightTurnMotorID = 32;
        public static final int BackLeftTurnMotorID = 33;
        public static final int BackRightTurnMotorID = 34;

        public static final int PigeonID = 1;

        /****************************** ARM ******************************/
        public static final int ArmMotorID = 41;
        public static final int WristMotorID = 42; // rev
        public static final int ElevatorMotorID = 43;
        public static final int CarriageMotorID = 45;
        public static final int WristEncoderID = 46;
        public static final int ArmEncoderID = 47;
        public static final int ElevatorHallEffect = 48;
        public static final int CarriageHallEffect = 49;

        /************************** ALGAE INTAKE ****************************/
        public static final int LiftMotorID = 51;
        public static final int LiftEncoderID = 52;
        public static final int AlgaeIntakeMotorID = 53;

        /***************************** GRIPPER *****************************/
        public static final int GripperMotorID = 61;

        /****************************** CLIMB ******************************/
        public static final int WinchMotorID = 58;
        public static final int ServoID = 0; // CHANGE?
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {
        
        public static final double NudgeSpeed = .25;

        public static final boolean GyroReversed = false;
        public static final double distanceToFrontOfRobot = Units.inchesToMeters(32/2-10); 
        public static final double TrackWidth = Units.inchesToMeters(22); // Distance between left and right wheels
        public static final double WheelBase = Units.inchesToMeters(20); // Distance between front and back wheels
        public static final double TotalWidth = Units.inchesToMeters(29);
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2), // front left
                new Translation2d(WheelBase / 2, -TrackWidth / 2), // front right
                new Translation2d(-WheelBase / 2, TrackWidth / 2), // back left
                new Translation2d(-WheelBase / 2, -TrackWidth / 2) // back right
        );

        public static final double WheelRadius = Units.inchesToMeters(2.0);
        public static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
        public static final double GearRatio = 7.13;

        public static final double VelocityConversionFactor = WheelCircumference / Constants.SecondsPerMinute
                / GearRatio;

        public static final double MaxAngularRadiansPerSecond = Math.PI; // 1/2 rotation per second
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE

        public static final double MaxAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxDriveableVelocity = 4.3; // m/s
        public static final double PhysicalMaxMetersPerSecond = NeoMaxRPM * VelocityConversionFactor;

        public static final double FrontLeftMagnetOffsetInRadians = 1.5171039327979088;
        public static final double FrontRightMagnetOffsetInRadians = 1.7456666082143784;
        public static final double BackLeftMagnetOffsetInRadians = -2.7626938149333;
        public static final double BackRightMagnetOffsetInRadians = -2.305568464100361;

        public static final double TimestepDurationInSeconds = 0.02;
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        /* DRIVE ENCODER */
        public static final double DriveKP =/*testing*/0; //0.04; // REQUIRES TUNING
        public static final double DriveKI =/*testing*/0; // 0.0015;
        public static final double DriveKD = 0;
        public static final double DriveIZone =/*testing*/0; // 0.15;
        public static final double DriveFF = 1.0 / PhysicalMaxMetersPerSecond;

        public static final double AutoAlignRotKP =/*testing*/0; // 0.08; // CHANGE
        public static final double AutoAlignRotKI = 0.0;
        public static final double AutoAlignRotKD = 0.0;

        public static final double AutoAlignNoteKP = /*testing*/0; //0.005; // CHANGE
        public static final double AutoAlignNoteKI = 0.0;
        public static final double AutoAlignNoteKD = 0;

        public static final double YawOverrideAlignNoteKP =/*testing*/0; // 0.0005; // NEED TO FIX
        public static final double YawOverrideAlignNoteKI = 0.0;
        public static final double YawOverrideAlignNoteKD = 0;

        public static final double TargetYawOverrideAlignNoteKP =/*testing*/0; // 0.004; // NEED TO FIX
        public static final double TargetYawOverrideAlignNoteKI = 0.0;
        public static final double TargetYawOverrideAlignNoteKD = 0;

        /* TURN ENCODER */
        public static final int CANcoderResolution = 4096;
        public static final double PositionConversionFactor = WheelCircumference / GearRatio;
        public static final double TurnKP =/*testing*/0; // 1; // NEED TO FIX
        public static final double TurnKI = 0;
        public static final double TurnKD = 0;

        public static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per
                                                                                   // revolution (rad/sec)
        public static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared
        public static final double MaxModuleMetersPerSecond = 4.5;

        public static final int StallLimit = 40;
        public static final int FreeLimit = 40;

        // public static final HolonomicPathFollowerConfig pathFollowerConfig = new
        // HolonomicPathFollowerConfig(
        // new PIDConstants(5.0, 0, 0), // Translation constants
        // new PIDConstants(5.0, 0, 0), // Rotation constants
        // MaxModuleMetersPerSecond,
        // flModuleOffset.getNorm(), // Drive base radius (distance from center to
        // furthest module)
        // new ReplanningConfig());
    }


    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class ArmConstants {

        // CHANGE - this is currently universal to all of the motors on the Arm? why?
        public static final double SupplyCurrentLimit = 40;
        
        public static final double CalibrationVelocity = -0.5; // -1.0;

        /* MOTION LIMITS */

        public static final double MaxHeightInches = 45; // CHANGE
        public static final double MaxCarriageHeight = 26.5; //inches from its base (0) position // will need to decrease this by 4 inches
        public static final double MaxElevatorHeight = 30; //(roughly) //inches from its base (0) position
        // public static final double StartingElevatorHeight = 24.75; // pretty sure we don't need this
        // public static final double StartingCarriageHeight = 24.75; // pretty sure we don't need this
        public static final double ArmMinimumHeight = 11; // carriage is 11 inches off of ground (not robo base)

        // so far these arent being used, but we should decided if we need to set limits on this movement
        public static final double MinArmAngle = -100; // CHANGE
        public static final double MaxArmAngle = 100; // CHANGE
        public static final double MinWristAngle = -5; // CHANGE
        public static final double MaxWristAngle = 100; // Max counterclockwise rotation


        /* ARM */

        public static final double ArmAngleCANcoderMagnetOffset = -0.058105; // rotations
        public static final double ArmAngleGearRatio = 86.4;
        public static final double ArmAngleDegreesPerMotorRotation = 360.0 / ArmAngleGearRatio;

        public static final double ArmAngleKP2 =/*testing*/ 12.0; // 32; // CHANGE
        public static TuneableParameter ArmAngleKP = new TuneableParameter(ArmAngleKP2, 0, 50, true, "TuneableParameter/Arm/PID/ArmAngleKP");
        public static final double ArmAngleKI2 =/*testing*/0; // 0.1; // CHANGE
        public static TuneableParameter ArmAngleKI = new TuneableParameter(ArmAngleKI2, 0, 1, true, "TuneableParameter/Arm/PID/ArmAngleKI");
        public static final double ArmAngleKD2 =/*testing*/0; // 0.05; // CHANGE
        public static TuneableParameter ArmAngleKD = new TuneableParameter(ArmAngleKD2, 0, 1, true, "TuneableParameter/Arm/PID/ArmAngleKD");
        public static final double ArmAngleKV2 = 9.0; // CHANGE
        public static TuneableParameter ArmAngleKV = new TuneableParameter(ArmAngleKV2, 0, 1, true, "TuneableParameter/Arm/PID/ArmAngleFF");
        public static final double ArmAngleKG2 = 0.25;
        public static TuneableParameter ArmAngleKG = new TuneableParameter(ArmAngleKG2, 0, 3, true, "TuneableParameter/Arm/PID/ArmAngleKG");
        public static final double ArmAngleKS2 = 0.14;
        public static TuneableParameter ArmAngleKS = new TuneableParameter(ArmAngleKS2, 0, 3, true, "TuneableParameter/Arm/PID/ArmAngleKG");
        
        public static final double ArmAngleCruiseSpeed = /*testing*/ 270.0/360.0; // .1; // RPS - CHANGE
        public static final double ArmAngleAcceleration = /*testing*/ ArmAngleCruiseSpeed * 4.0; // 12; // CHANGE
        public static final double ArmAngleJerk = /*testing*/ ArmAngleAcceleration * 4.0; // 40; // CHANGE
        

        /* ELEVATOR */

        public static final double ElevatorKP2 =/*testing*/0.4; // 32; // CHANGE
        public static TuneableParameter ElevatorKP = new TuneableParameter(ElevatorKP2, 0, 50, true, "TuneableParameter/Arm/PID/ElevatorKP");
        public static final double ElevatorKI2 =/*testing*/0; // 0.1; // CHANGE
        public static TuneableParameter ElevatorKI = new TuneableParameter(ElevatorKI2, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorKI");
        public static final double ElevatorKD2 =/*testing*/0.3; // 0.2; // CHANGE
        public static TuneableParameter ElevatorKD = new TuneableParameter(ElevatorKD2, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorKD");
        public static final double ElevatorKV2 =/*testing*/0.135; // 0.0; // CHANGE
        public static TuneableParameter ElevatorKV = new TuneableParameter(ElevatorKV2, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorFF");
        public static final double ElevatorKG2 =/*testing*/0.12; // 2.7; // CHANGE
        public static TuneableParameter ElevatorKG = new TuneableParameter(ElevatorKG2, 0, 3, true, "TuneableParameter/Arm/PID/ElevatorKG");
        public static final double ElevatorKS2 =/*testing*/0.05; // 0; // CHANGE
        public static TuneableParameter ElevatorKS = new TuneableParameter(ElevatorKS2, 0,1, true, "TuneableParameter/Arm/PID/ElevatorKS");
        public static final double ElevatorGearRatio = 5;//15;
        // public static final double ElevatorSprocketDiameter = 2.36; // inches // CHANGE
        public static final double ElevatorSprocketDiameter = 1.44;//2.16; // inches // CHANGE
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;
        public static final double RotationsPerElevatorInch = 1 / ElevatorSprocketCircumference * ElevatorGearRatio;

        public static final double ElevatorMaxInchesPerSec = KrakenX60MaxRPM / SecondsPerMinute / ElevatorGearRatio
                * ElevatorSprocketCircumference;
        public static final double ElevatorCruiseVelocity = 20.0; // CHANGE
        public static final double ElevatorAcceleration = 3.8 * ElevatorCruiseVelocity ; // CHANGE
        public static final double ElevatorJerk = 5*ElevatorAcceleration; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)

        public static final double ElevatorAlgaeRemovalVelocity = 8.0;
        // public static final double ElevatorAcceleration = 3.8 * ElevatorCruiseVelocity ; // CHANGE
        // public static final double ElevatorJerk = 5*ElevatorAcceleration; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)



        // public static final double CarriageKP =/*testing*/0.5; // 32; // CHANGE
        // public static TuneableParameter CarriageKPTP = new TuneableParameter(CarriageKP, 0, 50, true, "TuneableParameter/Arm/PID/ElevatorKP");
        // public static final double CarriageKI =/*testing*/0; // 0.1; // CHANGE
        // public static TuneableParameter CarriageKITP = new TuneableParameter(CarriageKI, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorKI");
        // public static final double CarriageKD =/*testing*/0.30; // 0.2; // CHANGE
        // public static TuneableParameter CarriageKDTP = new TuneableParameter(CarriageKD, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorKD");
        // public static final double CarriageKV =/*testing*/0.11; // 0.0; // CHANGE
        // public static TuneableParameter CarriageKVTP = new TuneableParameter(CarriageKV, 0, 1, true, "TuneableParameter/Arm/PID/ElevatorFF");
        // public static final double CarriageKG =/*testing*/0.1; // 2.7; // CHANGE
        // public static TuneableParameter CarriageKGTP = new TuneableParameter(CarriageKG, 0, 3, true, "TuneableParameter/Arm/PID/ElevatorKG");
        // public static final double CarriageKS =/*testing*/0.08; // 0; // CHANGE
        // public static TuneableParameter CarriageKSTP = new TuneableParameter(CarriageKS, 0,1, true, "TuneableParameter/Arm/PID/ElevatorKS");


        public static final double CarriageKP = 0.5;   // 32; // CHANGE
        public static final double CarriageKI = 0;    // 0.1; // CHANGE
        public static final double CarriageKD = 0.30; // 0.2; // CHANGE
//        public static final double CarriageKD = 0.00; // 0.2; // CHANGE
        public static final double CarriageKV = 0.11; // 0.0; // CHANGE
        public static final double CarriageKG = 0.1;  // 2.7; // CHANGE
        public static final double CarriageKS = 0.08; // 0; // CHANGE


        //  Values for Carriage now based on Mr K's spreadsheet
        public static final double CarriageGearRatio = 4.8;
        public static final double CarriageSprocketDiameter = 1.44; // inches // CHANGE
        public static final double CarriageSprocketCircumference = CarriageSprocketDiameter * Math.PI;
        public static final double RotationsPerCarriageInch = 1 / CarriageSprocketCircumference * CarriageGearRatio;
        
        public static final double CarriageMaxInchesPerSec = KrakenX60MaxRPM / SecondsPerMinute / CarriageGearRatio
                * CarriageSprocketCircumference;
        public static final double CarriageCruiseVelocity = 20.0; // CHANGE In inches per second
        public static final double CarriageAcceleration = 4 * CarriageCruiseVelocity ; // CHANGE In inches per second per second
        public static final double CarriageJerk = 5 * CarriageAcceleration ; // CHANGE - In Inches/s/s

        public static final double CarriageAlgaeRemovalVelocity = 8.0;



        // public static final double RotationsPerElevatorInch = ElevatorGearRatio / Units.metersToInches(ElevatorSprocketCircumference) / 2;


        /* WRIST */

        public static final double WristAngleCANcoderMagnetOffset = -0.219971;// old (not sure where this came from): 0.48583;
        public static final double WristAngleGearRatio = 20;
        public static final double WristAngleDegreesPerMotorRotation = 360.0 / WristAngleGearRatio;

        public static final double WristKP2 =/*testing*/0; // 2; // CHANGE
        public static TuneableParameter WristKP = new TuneableParameter(WristKP2, 0, 3, true, "TuneableParameter/Arm/PID/WristKP");
        public static final double WristKI2 =/*testing*/0; // 0.000; // CHANGE
        public static TuneableParameter WristKI = new TuneableParameter(WristKI2, 0, 1, true, "TuneableParameter/Arm/PID/WristKI");
        public static final double WristKD2=/*testing*/0; // 0.1; // CHANGE
        public static TuneableParameter WristKD = new TuneableParameter(WristKD2, 0, 1, true, "TuneableParameter/Arm/PID/WristKD");
        public static final double WristFF2 =/*testing*/0; // 0.0; // CHANGE
        public static TuneableParameter WristFF = new TuneableParameter(WristFF2, 0, 3, true, "TuneableParameter/Arm/PID/WrisFFP");
        public static final double WristIZone2 =/*testing*/0; // 0.15; // CHANGE
        public static TuneableParameter WristIZone = new TuneableParameter(WristIZone2, 0, 3, true, "TuneableParameter/Arm/PID/WristIZone");


        public static final double WristMaxAngularVelocity = 0.5 * Math.PI;
        public static final double WristMaxAngularAcceleration = 4 * WristMaxAngularVelocity;




        /* TOLERANCES */

        public static final double WristAngleTolerance = 20.0; // CHANGE wpk need to fix this after gains tuned
        public static final double ArmAngleTolerance = 5.0; // CHANGE
        // public static final double ArmAngleTolerance = 10.0; // change this back after climb testing
        public static final double ElevatorHeightTolerance = 0.5;// CHANGE
        public static final double CarriageHeightTolerance = 0.5;// CHANGE

        public static final double CalibratonVelocityTolerance = 0.25; // CHANGE

        public static final int CalibratonCurrentTolerance = 30; // CHANGE

     /* public static final double Level1ArmAngle = 10;
        public static final double Level2ArmAngle = 20;
        public static final double Level3ArmAngle = 30;
        public static final double Level4ArmAngle = 40;

        public static final double Level1WristAngle = 0; 
        public static final double Level2WristAngle = 90;
        public static final double Level3WristAngle = 90;
        public static final double Level4WristAngle = 90;

        public static final double Level1ElevatorHeight = 5;
        public static final double Level2ElevatorHeight = 10;
        public static final double Level3ElevatorHeight = 15;
        public static final double Level4ElevatorHeight = 20;

        public static final double Level1CarriageHeight = 5;
        public static final double Level2CarriageHeight = 10;
        public static final double Level3CarriageHeight = 15;
        public static final double Level4CarriageHeight = 20; 
    
        public static final double CoralWristAngle = 90; 
        public static final double CoralElevatorHeight = 25;
        public static final double CoralCarriageHeight = 22;
        public static final double CoralArmAngle = -20; 

        public static final double GrabbingCoralWristAngle = 90; 
        public static final double GrabbingCoralElevatorHeight = 25; 
        public static final double GrabbingCoralCarriageHeight = 22; 
        public static final double GrabbingCoralArmAngle = -20; 

        public static final double AutoCoralWristAngle = 90; 
        public static final double AutoCoralElevatorHeight = 25; 
        public static final double AutoCoralCarriageHeight = 22; 
        public static final double AutoCoralArmAngle = -20; 
        
        public static final double AlgaeHighArmAngle = 70; 
        public static final double AlgaeHighWristAngle = 0; 
        public static final double AlgaeHighElevatorHeight = 21; 
        public static final double AlgaeHighCarriageHeight = 23; 

        public static final double AlgaeLowWristAngle = 0; 
        public static final double AlgaeLowArmAngle = 25; 
        public static final double AlgaeLowElevatorHeight = 15; 
        public static final double AlgaeLowCarriageHeight = 13;  */
    }

    public static final class ClimbConstants {
//        public static final double WinchMotorGearRatio = 45; // CHANGE
        public static final double WinchMotorGearRatio = 45; // Correct per Mr. K 022725
        public static final double WinchSprocketDiameter = 0.75;    // Correct per Mr. K 022725
        public static final double WinchSprocketCircumference = WinchSprocketDiameter * Math.PI;
        public static final double RotationsPerWinchInch = 1 / WinchSprocketCircumference * WinchMotorGearRatio;
        public static final double WinchCableLenHarpoonHoriz = 18.5; // Length (in.) when the harpoon is pulled to fully horizontal
        public static final double WinchCableLenAfterClimb = 11; // Length when we have fully wound the Cable & bot climbed.
        //  We are assuming that at the start of a match, the winch is wound so that the harpoon is back inside the body.
        //      For convenience sake, we are just setting this to the same level as a full climb.
        public static final double WinchCableLenAtRest = WinchCableLenAfterClimb; // Length in inches, fully unwound with harpoon fwd & down
        //  Below measures the winch rotations required to move from the starting point, i.e. climbed, to horizontal harpoon
        //      As is, this is a negative value, i.e. we are unwinding the cable.
        public static final double WinchAttachRotations = (WinchCableLenAtRest - WinchCableLenHarpoonHoriz)*RotationsPerWinchInch; 
        //  Below measures the winch rotations required to move from horizontal harpoon to fully back, i.e. climbed.
        public static final double WinchClimbRotations = (WinchCableLenHarpoonHoriz - WinchCableLenAfterClimb)*RotationsPerWinchInch;

        public static final double WinchMaxInchesPerSec = KrakenX60MaxRPM / SecondsPerMinute / WinchMotorGearRatio
                * WinchSprocketCircumference;
        public static final double WinchKP2 = 0;
        public static TuneableParameter WinchKP = new TuneableParameter(WinchKP2, 0, 1, true, "TuneableParameter/Climb/PID/WinchKP");
        public static final double WinchKI2 = 0;
        public static TuneableParameter WinchKI = new TuneableParameter(WinchKI2, 0, 1, true, "TuneableParameter/Climb/PID/WinchKI");
        public static final double WinchKD2 = 0;
        public static TuneableParameter WinchKD = new TuneableParameter(WinchKD2, 0, 1, true, "TuneableParameter/Climb/PID/WinchDP");
        public static final double WinchFF2 = 0;
        public static TuneableParameter WinchFF = new TuneableParameter(WinchFF2, 0, 1, true, "TuneableParameter/Climb/PID/WinchKP");
        public static final double WinchKG2 = 0;
        public static TuneableParameter WinchKG = new TuneableParameter(WinchKG2, 0, 1, true, "TuneableParameter/Climb/PID/WinchKG");


        public static final double WinchCruiseRotationsPerSec = 20;
        public static final double WinchAcceleration = WinchCruiseRotationsPerSec * 4;
        public static final double WinchJerk = WinchAcceleration * 10;

        public static final double SupplyCurrentLimit = 100; // CHANGE

        public static final double CageAngle2 = WinchAttachRotations * 360;
        public static TuneableParameter UnwoundAngle = new TuneableParameter(CageAngle2, 0, 180, true, "TuneableParameter/Climb/CageAngle");
        
        public static final double WinchedAngle2 = WinchClimbRotations * 360;
        public static TuneableParameter WoundAngle = new TuneableParameter(WinchedAngle2, 0, 180, true, "TuneableParameter/Climb/WinchedAngle");

        //  A value between 1.0 (fully retracted) and 0.0 (fully extended)
        //      For initial testing, start with a low value.  Movement required is .25-.375 inches.
        //      Not sure how this translates for this specific device, but full extension is a bit more than 1 in.
        public static final double ServoExtendedPos = 0.65;  // Low initial value for testing.  Increase in small increments
                                                            //  as needed.
        public static final double ServoRetractedPos = 1.0;  // Low initial value for testing.  Increase in small increments

        public static final double WinchTolerance = 5;  //  In degrees..Get the winch rotation within +/- this many degrees

    }

    public static final class GripperConstants {
        public static final double GripperKP2 =/*testing*/0.0; // 32; // CHANGE
        public static TuneableParameter GripperKP = new TuneableParameter(GripperKP2, 0, 100, true, "TuneableParameter/Gripper/PID/GripperKP");
        public static final double GripperKI2 =/*testing*/0; // 0.001; // CHANGE
        public static TuneableParameter GripperKI = new TuneableParameter(GripperKI2, 0, 1, true, "TuneableParameter/Gripper/PID/GripperKI");
        public static final double GripperKD2 =/*testing*/0; // 0; // CHANGE
        public static TuneableParameter GripperKD = new TuneableParameter(GripperKD2, 0, 10, true, "TuneableParameter/Gripper/PID/GripperKD");
        public static final double GripperFF2 =/*testing*/1; // 0; // CHANGE
        public static TuneableParameter GripperFF = new TuneableParameter(GripperFF2, 0, 100, true, "TuneableParameter/Gripper/PID/GripperFF");
        public static final double GripperIZone2 =/*testing*/0; // 0.15; // CHANGE
        public static TuneableParameter GripperIZone = new TuneableParameter(GripperIZone2, 0, 1, true, "TuneableParameter/Gripper/PID/GripperIZone2");

        public static final double IntakeSpeed2 = 1; // CHANGE
        public static TuneableParameter IntakeSpeed = new TuneableParameter(IntakeSpeed2, 0, 1.5, true, "TuneableParameter/Gripper/IntakeSpeed");

        public static final double GearRatio = 12.0 ;
        public static final double EjectSpeed2 = -3 * GearRatio; // CHANGE
        public static TuneableParameter EjectSpeed = new TuneableParameter(EjectSpeed2, 0, 1000, true, "TuneableParameter/Gripper/EjectSpeed");


        public static final double currentOfIntakedCoral2 = 2000000; // Change with testing. 
        public static TuneableParameter currentOfIntakedCoral = new TuneableParameter(currentOfIntakedCoral2, 0, 4000000, true, "TuneableParameter/Gripper/currentOfIntakedCoral");

        public static final double locationOfGripperToRobotX = Units.inchesToMeters(1);
    }

    public static final class AlgaeConstants {
        public static final double LiftKP2 =/*testing*/1.5; // 0.04; // CHANGE
        public static TuneableParameter LiftKP = new TuneableParameter(LiftKP2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/LiftKP");
        public static final double LiftKI2 =/*testing*/0; // 0.06; // CHANGE
        public static TuneableParameter LiftKI = new TuneableParameter(LiftKI2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/LiftKI");
        public static final double LiftKD2 =/*testing*/0.1; // 0; // CHANGE
        public static TuneableParameter LiftKD = new TuneableParameter(LiftKD2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/LiftKD");
        public static final double LiftKV2 =/*testing*/6.0; // 0.13; // CHANGE
        public static TuneableParameter LiftKV = new TuneableParameter(LiftKV2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/LiftFF");
        public static final double LiftKG2 =/*testing*/0; // 0; // CHANGE
        public static TuneableParameter LiftKG = new TuneableParameter(LiftKG2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/LiftKG");

        public static final double LiftIZone =/*testing*/0; // 0.15; // CHANGE

        public static final double IntakeKS2 = 0; // CHANGE
        public static TuneableParameter IntakeKS = new TuneableParameter(IntakeKS2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeKS");

        public static final double IntakeKV2 =/*testing*/2.0; // 0.13; // CHANGE
        public static TuneableParameter IntakeKV = new TuneableParameter(IntakeKV2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeFF");
        public static final double IntakeKA2 = 0; // CHANGE
        public static TuneableParameter IntakeKA = new TuneableParameter(IntakeKA2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeKA");
        public static final double IntakeKP2 = 0; // CHANGE
        public static TuneableParameter IntakeKP = new TuneableParameter(IntakeKP2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeKP");
        public static final double IntakeKI2 = 0; // CHANGE
        public static TuneableParameter IntakeKI = new TuneableParameter(IntakeKI2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeKI");
        public static final double IntakeKD2 = 0; // CHANGE
        public static TuneableParameter IntakeKD = new TuneableParameter(IntakeKD2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeKD");
        public static final double IntakeIZone2 =/*testing*/0; // 0.15; // CHANGE
        public static TuneableParameter IntakeIZone = new TuneableParameter(IntakeIZone2, 0, 1, true, "TuneableParameter/AlgaeIntake/PID/IntakeIZone");

        public static final double IntakeSpeed2 = 4.0; // CHANGE
        public static TuneableParameter IntakeSpeed = new TuneableParameter(IntakeSpeed2, 0, 25, true, "TuneableParameter/AlgaeIntake/IntakeSpeed");
        public static final double EjectSpeed2 = -2; // CHANGE
        public static TuneableParameter EjectSpeed = new TuneableParameter(EjectSpeed2, -25, 0, true, "TuneableParameter/AlgaeIntake/EjectSpeed");

        public static final double LiftCruiseRotationsPerSec = 0.25; // CHANGE
        public static final double LiftAcceleration = 2*LiftCruiseRotationsPerSec ; // CHANGE
        public static final double LiftJerk = 4*LiftAcceleration; // CHANGE
        
        public static final double IntakeAcceleration = 400; // CHANGE
        public static final double IntakeJerk = 4000; // CHANGE
        public static final double LiftAngleTolerance = 3.5;
        public static final double SupplyCurrentLimit = 30; // CHANGE

        public static final double retractedAngle2 = -8 ; 
        public static final double deployedAngle2 = 56 + retractedAngle2; 

        public static final double LiftGearRatio = 60.0/ 14.0 ;

        public static TuneableParameter deployedAngle = new TuneableParameter(deployedAngle2, 0, 180, true, "TuneableParameter/AlgaeIntake/DeployedAngle"); //CHANGE
        public static TuneableParameter retractedAngle = new TuneableParameter(retractedAngle2,  -180,180, true, "TuneableParameter/AlgaeIntake/RestedAngle");
    }

    public static final class AutoConstants {
        public static TuneableParameter coralIsVisible = new TuneableParameter(0, 0, 1, true, "Auto/CoralIsVisible");

        public static final double MaxSpeedMetersPerSecond = DriveConstants.MaxModuleMetersPerSecond / 4; // CHANGE
        public static final double MaxAngularSpeedRadiansPerSecond = DriveConstants.PhysicalMaxAngularSpeedRadiansPerSecond
                / 10; // Default is 540 degress
        public static final double kPXController =/*testing*/0; // 1;
        public static final double kPYController =/*testing*/0; // 1;
        public static final double kPThetaController =/*testing*/0; // 1;

        public static final double MaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // default: 720 deg

    }

    public final class LogitechDAConstants {
        public static final int LeftStickX = 0;
        public static final int LeftStickY = 1;
        public static final int RightStickX = 2;
        public static final int RightStickY = 3;
        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int ButtonA = 2;
        public static final int ButtonB = 3;
        public static final int ButtonX = 1;
        public static final int ButtonY = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int BackButton = 9;
        public static final int StartButton = 10;
        public static final int LeftStick = 11;
        public static final int RightStick = 12;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class LogitechExtreme3DConstants {
        public static final int AxisX = 0;
        public static final int AxisY = 1;
        public static final int AxisZRotate = 2;
        public static final int Slider = 3;
        public static final int Trigger = 1;
        public static final int ButtonStick = 2;
        public static final int Button3 = 3;
        public static final int Button4 = 4;
        public static final int Button5 = 5;
        public static final int Button6 = 6;
        public static final int Button7 = 7;
        public static final int Button8 = 8;
        public static final int Button9 = 9;
        public static final int Button10 = 10;
        public static final int Button11 = 11;
        public static final int Button12 = 12;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class RadioMasterConstants {
        public static final int LeftGimbalX = 0;
        public static final int LeftGimbalY = 1;
        public static final int RightGimbalX = 3;
        public static final int RightGimbalY = 2;
        public static final int SliderF = 5;
        public static final int SliderE = 4;
        public static final int SliderC = 6;
        public static final int ButtonD = 2;
        public static final int ButtonA = 1;
        public static final double FowardAxisAttenuation = 1.0;
        public static final double LateralAxisAttenuation = 1.0;
        public static final double YawAxisAttenuation = 0.6;
    }

    public final class XboxControllerConstants {
        // public static final int LeftStickX = 0;
        // public static final int LeftStickY = 1;
        // public static final int LeftTrigger = 2;
        // public static final int RightTrigger = 4;
        // public static final int RightStickX = 4;
        // public static final int RightStickY = 5;
        public static final int ButtonA = 1;
        public static final int ButtonB = 2;
        public static final int ButtonX = 3;
        public static final int ButtonY = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int WindowButton = 7;
        // public static final int BackButton = 7;
        public static final int HamburgerButton = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;

        public static final int POVUp = 0 ;
        public static final int POVDown = 180 ;
        public static final int POVLeft = 270 ;
        public static final int POVRight = 90 ;


    }

    public final class FieldConstants {
        public static final double reefSideLengthInches = 37; 
        public static final double reefOffsetInches = 7.5; // goes to the middle of the Side
        public static final double reefOffsetMeters = 0.025406 * reefOffsetInches;
    }
}

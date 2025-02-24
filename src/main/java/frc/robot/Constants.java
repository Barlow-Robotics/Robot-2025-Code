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

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/
    public static final class VisionConstants {
        public static final int CameraLightID = 0; // NEED TO FIX/CHANGE
        public static final String PoseCameraName = "Global_Shutter_Camera";
        public static final String TargetCameraName = "Arducam_OV9281_USB_Camera";

        public static final PoseStrategy PrimaryVisionStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
        public static final PoseStrategy FallbackVisionStrategy = PoseStrategy.LOWEST_AMBIGUITY;

        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        // NEED TO FIX: wpk need to update these to be more exact.
        public static final Transform3d PoseCameraToRobot = new Transform3d(
                new Translation3d(0.0, -Units.inchesToMeters(DriveConstants.TrackWidth / 2), Units.inchesToMeters(23)),
                new Rotation3d(0, Units.degreesToRadians(5), 0));
        public static final Transform3d RobotToPoseCamera = PoseCameraToRobot.inverse();

        public static final Transform3d TargetCamToRobot =
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(DriveConstants.TrackWidth/2), Units.inchesToMeters(23)), 
                    new Rotation3d(0, Units.degreesToRadians(5), 0));
        public static final Transform3d RobotToTargetCam = TargetCamToRobot.inverse();

        // // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout FieldTagLayout = AprilTagFields.k2025ReefscapeAndyMark
                .loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        // constants for vision-calculated speaker shooting - LMT
        public static final int RedSpeakerCenterAprilTagID = 4;
        public static final int BlueSpeakerCenterAprilTagID = 7;
        public static final int NullAprilTagID = -1;
        public static final double InvalidAngle = -361;
        public static final double NoTargetDistance = -1;

        public static final int[] blueAprilTagListReef = {17, 18, 19, 20, 21, 22};
        public static final int[] redAprilTagListReef = {6, 7, 8, 9, 10, 11};


        public static final int[] blueAprilTagListCoralStation = {12, 13};
        public static final int[] redAprilTagListCoralStation = {1, 2};

    
        public static final double NoteAlignPixelTolerance = 250; // NEED TO CHANGE

    }

    public static final class ElectronicsIDs {

        public static final int DriverControllerPort = 1;
        public static final int OperatorControllerPort = 2;

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

        /************************** ALGAE INTAKE ****************************/
        public static final int LiftMotorID = 51;
        public static final int LiftEncoderID = 52;
        public static final int AlgaeIntakeMotorID = 53;

        /***************************** GRIPPER *****************************/
        public static final int GripperMotorID = 61;

        /****************************** CLIMB ******************************/
        public static final int WinchMotorID = 71;
        public static final int ServoID = 1; // CHANGE?
    }

    /***************************************************************************/
    /***************************************************************************/
    /***************************************************************************/

    public static final class DriveConstants {

        public static final boolean GyroReversed = false;
        public static final double distanceToFrontOfRobot = Units.inchesToMeters(22); 
        public static final double TrackWidth = Units.inchesToMeters(22); // Distance between left and right wheels
        public static final double WheelBase = Units.inchesToMeters(20); // Distance between front and back wheels
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

        /* MOTION LIMITS */

        public static final double MaxHeightInches = 45; // CHANGE
        public static final double MaxCarriageHeight = 26.5; //inches from its base (0) position // will need to decrease this by 4 inches
        public static final double MaxElevatorHeight = 30; //(roughly) //inches from its base (0) position
        // public static final double StartingElevatorHeight = 24.75; // pretty sure we don't need this
        // public static final double StartingCarriageHeight = 24.75; // pretty sure we don't need this
        public static final double ArmMinimumHeight = 11; // carriage is 11 inches off of ground (not robo base)

        // so far these arent being used, but we should decided if we need to set limits on this movement
        public static final double MinArmAngle = 0; // CHANGE
        public static final double MaxArmAngle = 0; // CHANGE
        public static final double MinWristAngle = 0; // CHANGE
        public static final double MaxWristAngle = 90; // Max counterclockwise rotation


        /* ARM */

        public static final double ArmAngleCANcoderMagnetOffset = -0.058105;

        public static final double ArmAngleKP =/*testing*/0; // 32; // CHANGE
        public static final double ArmAngleKI =/*testing*/0; // 0.1; // CHANGE
        public static final double ArmAngleKD =/*testing*/0; // 0.05; // CHANGE
        public static final double ArmAngleFF = 0; // CHANGE
        public static final double ArmAngleKG =/*testing*/0; // 0.29; // CHANGE
        public static final double ArmAngleCruiseSpeed =/*testing*/0; // .1; // rotations per sec // CHANGE
        public static final double ArmAngleAcceleration =/*testing*/0; // 12; // CHANGE
        public static final double ArmAngleJerk =/*testing*/0; // 40; // CHANGE
        

        /* ELEVATOR */

        public static final double ElevatorKP =/*testing*/0; // 32; // CHANGE
        public static final double ElevatorKI =/*testing*/0; // 0.1; // CHANGE
        public static final double ElevatorKD =/*testing*/0; // 0.2; // CHANGE
        public static final double ElevatorFF =/*testing*/0; // 0.0; // CHANGE
        public static final double ElevatorKG =/*testing*/0; // 2.7; // CHANGE
        public static final double ElevatorKS =/*testing*/0; // 0; // CHANGE
        public static final double ElevatorGearRatio = 5;//15;
        // public static final double ElevatorSprocketDiameter = 2.36; // inches // CHANGE
        public static final double ElevatorSprocketDiameter = 1.44;//2.16; // inches // CHANGE
        public static final double ElevatorSprocketCircumference = ElevatorSprocketDiameter * Math.PI;
        public static final double RotationsPerElevatorInch = 1 / ElevatorSprocketCircumference * ElevatorGearRatio;

        //  Values for Carriage now based on Mr K's spreadsheet
        public static final double CarriageGearRatio = 4.8;
        public static final double CarriageSprocketDiameter = 1.44; // inches // CHANGE
        public static final double CarriageSprocketCircumference = CarriageSprocketDiameter * Math.PI;
        public static final double RotationsPerCarriageInch = 1 / CarriageSprocketCircumference * CarriageGearRatio;
        
        // public static final double RotationsPerElevatorInch = ElevatorGearRatio / Units.metersToInches(ElevatorSprocketCircumference) / 2;

        public static final double ElevatorMaxInchesPerSec = Falcon500MaxRPM / SecondsPerMinute / ElevatorGearRatio
                * ElevatorSprocketCircumference;
        public static final double ElevatorCruiseInchesPerSec = 10; // CHANGE
        public static final double ElevatorInchesPerSecPerSec = 10; // CHANGE
        public static final double ElevatorJerk = 800; // CHANGE - Target jerk of 1600 rps/s/s (0.1 seconds)

        /* WRIST */

        public static final double WristAngleCANcoderMagnetOffset = -0.219971;// old (not sure where this came from): 0.48583;

        public static final double WristKP =/*testing*/0; // 2; // CHANGE
        public static final double WristKI =/*testing*/0; // 0.000; // CHANGE
        public static final double WristKD =/*testing*/0; // 0.1; // CHANGE
        public static final double WristFF =/*testing*/0; // 0.0; // CHANGE
        public static final double WristIZone =/*testing*/0; // 0.15; // CHANGE


        /* TOLERANCES */

        public static final double WristAngleTolerance = 1.5; // CHANGE
        public static final double ArmAngleTolerance = 3.5; // CHANGE
        public static final double ElevatorHeightTolerance = 0.25;// CHANGE
        public static final double CarriageHeightTolerance = 0.25;// CHANGE

     /* public static final double Level1ArmAngle = 10; // CHANGE
        public static final double Level2ArmAngle = 20; // CHANGE
        public static final double Level3ArmAngle = 30; // CHANGE
        public static final double Level4ArmAngle = 40; // CHANGE

        public static final double Level1WristAngle = 0; // CHANGE
        public static final double Level2WristAngle = 90; // CHANGE
        public static final double Level3WristAngle = 90; // CHANGE
        public static final double Level4WristAngle = 90; // CHANGE

        public static final double Level1ElevatorHeight = 5; // CHANGE
        public static final double Level2ElevatorHeight = 10; // CHANGE
        public static final double Level3ElevatorHeight = 15; // CHANGE
        public static final double Level4ElevatorHeight = 20; // CHANGE

        public static final double Level1CarriageHeight = 5; // CHANGE
        public static final double Level2CarriageHeight = 10; // CHANGE
        public static final double Level3CarriageHeight = 15; // CHANGE
        public static final double Level4CarriageHeight = 20; // CHANGE 
    
        public static final double CoralWristAngle = 90; // CHANGE      
        public static final double CoralElevatorHeight = 25; // CHANGE
        public static final double CoralCarriageHeight = 22; // CHANGE
        public static final double CoralArmAngle = -20; // CHANGE

        public static final double GrabbingCoralWristAngle = 90; // CHANGE
        public static final double GrabbingCoralElevatorHeight = 25; // CHANGE
        public static final double GrabbingCoralCarriageHeight = 22; // CHANGE
        public static final double GrabbingCoralArmAngle = -20; // CHANGE

        public static final double AutoCoralWristAngle = 90; // CHANGE
        public static final double AutoCoralElevatorHeight = 25; // CHANGE
        public static final double AutoCoralCarriageHeight = 22; // CHANGE
        public static final double AutoCoralArmAngle = -20; // CHANGE
        
        public static final double AlgaeHighArmAngle = 70; // CHANGE
        public static final double AlgaeHighWristAngle = 0; // CHANGE
        public static final double AlgaeHighElevatorHeight = 21; // CHANGE
        public static final double AlgaeHighCarriageHeight = 23; // CHANGE

        public static final double AlgaeLowWristAngle = 0; // CHANGE
        public static final double AlgaeLowArmAngle = 25; // CHANGE
        public static final double AlgaeLowElevatorHeight = 15; // CHANGE
        public static final double AlgaeLowCarriageHeight = 13; // CHANGE */
    }

    public static final class ClimbConstants {
        public static final double WinchMotorGearRatio = 1; // CHANGE

        public static final double WinchKP = 0;
        public static final double WinchKI = 0;
        public static final double WinchKD = 0;
        public static final double WinchFF = 0;
        public static final double WinchKG = 0;

        public static final double WinchCruiseRotationsPerSec = 0;
        public static final double WinchAcceleration = 0;
        public static final double WinchJerk = 0;

        public static final double SupplyCurrentLimit = 100; // CHANGE

        public static final double CageAngle = 0;
        public static final double WinchedAngle = 0;
        public static final int ServoExtendedPos = 1;

        public static final double WinchTolerance = 0;

        public static final int ServoTolerance = 0;



    }

    public static final class GripperConstants {
        public static final double GripperKP =/*testing*/0; // 32; // CHANGE
        public static final double GripperKI =/*testing*/0; // 0.001; // CHANGE
        public static final double GripperKD =/*testing*/0; // 0; // CHANGE
        public static final double GripperFF =/*testing*/0; // 0; // CHANGE
        public static final double GripperIZone =/*testing*/0; // 0.15; // CHANGE

        public static final double IntakeSpeed = 1; // CHANGE
        public static final double EjectSpeed = -1; // CHANGE

        public static final double currentOfIntakedCoral = 2000000; // Change with testing. 
    }

    public static final class AlgaeConstants {
        public static final double LiftKP =/*testing*/0; // 0.04; // CHANGE
        public static final double LiftKI =/*testing*/0; // 0.06; // CHANGE
        public static final double LiftKD =/*testing*/0; // 0; // CHANGE
        public static final double LiftFF =/*testing*/0; // 0.13; // CHANGE
        public static final double LiftKG =/*testing*/0; // 0; // CHANGE
        public static final double LiftIZone =/*testing*/0; // 0.15; // CHANGE

        public static final double IntakeKS = 0; // CHANGE
        public static final double IntakeFF =/*testing*/0; // 0.13; // CHANGE
        public static final double IntakeKA = 0; // CHANGE
        public static final double IntakeKP = 0; // CHANGE
        public static final double IntakeKI = 0; // CHANGE
        public static final double IntakeKD = 0; // CHANGE
        public static final double IntakeIZone =/*testing*/0; // 0.15; // CHANGE

        public static final double IntakeSpeed2 = 2; // CHANGE
        public static TuneableParameter IntakeSpeed = new TuneableParameter(IntakeSpeed2, 5, 0, true, "AlgaeIntake/IntakeSpeed");
        public static final double EjectSpeed2 = -2; // CHANGE
        public static TuneableParameter EjectSpeed = new TuneableParameter(EjectSpeed2, 0, -5, true, "AlgaeIntake/EjectSpeed");

        public static final double LiftAcceleration = 12; // CHANGE
        public static final double LiftJerk = 40; // CHANGE
        public static final double LiftCruiseRotationsPerSec = 3; // CHANGE
        
        public static final double IntakeAcceleration = 400; // CHANGE
        public static final double IntakeJerk = 4000; // CHANGE
        public static final double LiftAngleTolerance = 3.5;
        public static final double SupplyCurrentLimit = 30; // CHANGE

        public static final double deployedAngle2 = 60; 
        public static final double restedAngle2 = 0; 

        public static TuneableParameter deployedAngle = new TuneableParameter(deployedAngle2, 180,0, true, "AlgaeIntake/DeployedAngle"); //CHANGE
        public static TuneableParameter restedAngle = new TuneableParameter(restedAngle2, 180 ,0, true, "AlgaeIntake/RestedAngle");
    }

    public static final class AutoConstants {
        public static TuneableParameter coralIsVisible = new TuneableParameter(0, 1, 0, true, "Auto/CoralIsVisible");

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
        public static final int LeftStickX = 0;
        public static final int LeftStickY = 1;
        public static final int LeftTrigger = 2;
        public static final int RightTrigger = 4;
        public static final int RightStickX = 4;
        public static final int RightStickY = 5;
        public static final int ButtonA = 1;
        public static final int ButtonB = 2;
        public static final int ButtonX = 3;
        public static final int ButtonY = 4;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        // public static final int BackButton = 7;
        public static final int HamburgerButton = 8;
        public static final int LeftStick = 9;
        public static final int RightStick = 10;
        public static final int WindowButton = 7;
        public static final double ForwardAxisAttenuation = -0.5;
        public static final double LateralAxisAttenuation = 0.5;
        public static final double YawAxisAttenuation = 0.5;
    }

    public final class FieldConstants {
        public static final double reefSideLengthInches = 37; 
        public static final double reefOffsetInches = reefSideLengthInches/4; // goes to the middle of the Side
        public static final double reefOffsetMeters = 0.025406 * reefOffsetInches;
    }
}

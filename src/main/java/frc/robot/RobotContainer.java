
package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.*;
import choreo.Choreo;
// import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.config.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechExtreme3DConstants;
// import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.autonomous.DynamicChoreoCommand;
import frc.robot.autonomous.DynamicChoreo;
import frc.robot.autonomous.DynamicPathPlanner;
import frc.robot.commands.DriveRobot;
// <<<<<<< Updated upstream
// import edu.wpi.first.wpilibj2.command.Command; 
// import javax.swing.SwingUtilities; 
// // import frc.robot.RobotCommunicator;
// =======
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotCommunicator;
// >>>>>>> Stashed changes
import javax.swing.*;
import java.awt.Point;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.ObjectInputFilter.Config;
import java.util.ArrayList;
// import frc.robot.commands.DriveRobotWithAprilTagAlign;
// import frc.robot.commands.DriveRobotWithNoteAlign;
// import frc.robot.commands.PivotToPoint;
// import frc.robot.commands.ReverseFloorIntake;
// import frc.robot.commands.DriveRobotWithAlign;
// import frc.robot.commands.SetShooterMountPosition;
// import frc.robot.commands.StartClimbing;
// import frc.robot.commands.StartShooterIntake;
// import frc.robot.commands.StopShooterIntake;
import frc.robot.subsystems.Drive;
// import frc.robot.subsystems.FloorIntake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.ShooterMount;
// import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* CONTROLLERS */
    public static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger moveToSpeakerButton; // button x
    private Trigger moveToAmpButton; // button y
    private Trigger moveToSourceButton; // left stick
    private Trigger moveToFloorButton; // left bumper
    private Trigger moveToFerryButton; // hamburger

    private Trigger climbButton; // button a
    private Trigger piviotToPoint;
    private Trigger moveToCoralButton;
    private Trigger moveToRightButton;

    // private Trigger climbAbortButton; // right stick

    // private Trigger toggleLEDsButton; // hamburger
    // private Trigger LEDHumanSourceButton;
    // private Trigger LEDHumanFloorButton;

    private Trigger shootIntakeButton; // trigger
    private Trigger reverseFloorIntakeButton; // driver button 7

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9
    private Trigger lockInPlaceButton;

    private PIDController noteYawPID;
    private PIDController targetYawPID;

    public final Drive driveSub;
    public final Vision visionSub;
    public final Underglow underglowSub;
    /* AUTO */

    private SendableChooser<Command> autoChooser;
    boolean moveToCoral;
    boolean goToRight;


    public Pose2d reefAutoTargetPose = new Pose2d();
    // private final RobotCommunicator communicator; 
    // private RobotController robotController;
    public RobotContainer() {
        goToRight = false;
        // communicator = new RobotCommunicator(); // Initialize GUI on the Swing Event Dispatch Thread 
        // SwingUtilities.invokeLater(() -> { robotController = new RobotController(communicator); 
        // SwingUtilities.invokeLater(() -> { 
        //     robotController = new RobotController(communicator); 
        //     JFrame frame = new JFrame("Robot Controller"); 
        //     frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); 
        //     frame.add(robotController); frame.pack(); frame.setVisible(true); 
        // });
        moveToCoral = false; 
        visionSub = new Vision();
        driveSub = new Drive(visionSub);
        underglowSub = new Underglow();
        noteYawPID = new PIDController(
                DriveConstants.YawOverrideAlignNoteKP,
                DriveConstants.YawOverrideAlignNoteKI,
                DriveConstants.YawOverrideAlignNoteKD);
        noteYawPID.setSetpoint(0.0);

        targetYawPID = new PIDController(
                DriveConstants.TargetYawOverrideAlignNoteKP,
                DriveConstants.TargetYawOverrideAlignNoteKI,
                DriveConstants.TargetYawOverrideAlignNoteKD);
        targetYawPID.setSetpoint(0.0);

        configureBindings();
        driveSub.setDefaultCommand(
                new DriveRobot(
                        driveSub,
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider), 
                        () -> lockInPlaceButton.getAsBoolean(),
                        true));

        configurePathPlanner();
                    }

    private void configureBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** DRIVE *****************/

        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        restartGyroButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

        moveToCoralButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button8);
        moveToCoralButton.onTrue(new InstantCommand(() -> changeCoralVision(true))).onFalse(new InstantCommand(() -> changeCoralVision(false)));

        moveToRightButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button7);
        moveToRightButton.onTrue(new InstantCommand(() -> changeToRight(true))).onFalse(new InstantCommand(() -> changeToRight(false)));

        lockInPlaceButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button6);
        // lockInPlaceButton.getAsBoolean();

        // lockInPlaceButton.onTrue(new InstantCommand(() -> (true))).onFalse(new InstantCommand(() -> changeToRight(false)));

    }


    public void configureTeleOpPathPlanner() {
        RobotConfig config;

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
        }
            
        AutoBuilder.configure(
                driveSub::getPose, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveSub.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.5) // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return false;
                    }
                    return false;
                },
                driveSub);
    }


    public void configurePathPlanner() {
        
        RobotConfig config;

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace(); 
            config = null;
        }
            
        /* PATHPLANNER INIT */
        AutoBuilder.configure(
                driveSub::getPose, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveSub.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.5) // Rotation PID constants
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent() && DriverStation.isAutonomous()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false; 
                },
                driveSub);

        // NamedCommands.registerCommand("StartShooterIntake", startShooterIntakeCmd);
        // NamedCommands.registerCommand("StopShooterIntake", stopShooterIntakeCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionAmp", setShooterPosAmpCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionSpeaker", setShooterPosSpeakerCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionFloor", setShooterPosFloorCmd);
        // NamedCommands.registerCommand("start_intake_shoot", new SequentialCommandGroup(setShooterPosSpeakerCmd, startShooterIntakeCmd));

        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        //     if (alliance.get() == DriverStation.Alliance.Red) {
        //         NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommandRED);
        //         NamedCommands.registerCommand("PivotToCenterNote", piviotToCenterNoteCommandRED);
        //         NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommandRED);
        //         NamedCommands.registerCommand("PivotToStageNote", piviotToStageNoteCommandRED);
        //     } else {
        //         NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommand);
        //         NamedCommands.registerCommand("PivotToCenterNote", piviotToCenterNoteCommand);
        //         NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommand);
        //         NamedCommands.registerCommand("PivotToStageNote", piviotToStageNoteCommand);
        //     }
        // }



        autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio and
                                                      // delete them there
        SmartDashboard.putData("Selected Auto", autoChooser);
        autoChooser.setDefaultOption("VisionTest", new PathPlannerAuto("TestVision"));
        // autoChooser.addOption("Routine A", new DynamicPathPlanner("Routine A", visionSub));
        // autoChooser.addOption("Routine B", new DynamicPathPlanner("Routine B", visionSub));
        // autoChooser.addOption("Routine C", new DynamicChoreo("Routine C", visionSub, driveSub));
        // autoChooser.addOption("Routine D", new DynamicChoreoCommand("Routine D", visionSub, driveSub));
        

        autoChooser.addOption("ChoreoTEST", driveSub.ChoreoAuto("Straight Line Path"));
        // autoChooser.addOption("Test1", driveSub.ChoreoAutoWithoutReset("Test1"));
        // autoChooser.addOption("Test2", driveSub.ChoreoAuto("Test2"));
        // autoChooser.addOption("Test3", driveSub.ChoreoAuto("Test3"));
        // autoChooser.addOption("Test4", driveSub.ChoreoAuto("Test4"));
        // autoChooser.addOption("Test5", driveSub.ChoreoAuto("Test5"));
        // autoChooser.addOption("Test6", driveSub.ChoreoAuto("Test6"));


        Shuffleboard.getTab("Match").add("Path Name", autoChooser);

        /* LOGGING */
        // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        PathPlannerLogging.setLogCurrentPoseCallback(
                (currentPose) -> {
                    Logger.recordOutput("Odometry/CurrentPose", currentPose);
                });
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    // public Optional<Rotation2d> getRotationTargetOverride(){

    //     Optional<Rotation2d> result = Optional.empty() ;

    //     // if (RobotState.isAutonomous()) {

    //     //     // if ( shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake ) {
    //     //     //     if (visionSub.noteIsVisible() && Math.abs(visionSub.getNoteDistanceFromCenter()) < Constants.VisionConstants.NoteAlignPixelTolerance) {
    //     //     //         double yaw = visionSub.getNoteDistanceFromCenter();
    //     //     //         double rotDelta = noteYawPID.calculate(yaw);
    //     //     //         result = Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotDelta)) ) ;
    //     //     //     } else {
    //     //     //         noteYawPID.reset();
    //     //     //     }
    //     //     // } else if (shooterMountSub.getShooterMountState() == ShooterMountState.Speaker && !visionSub.allDetectedTargets.isEmpty()) {
    //     //     //     var bestTarget = visionSub.getBestTrackableTarget() ;
    //     //     //     if (bestTarget.isPresent()) {
    //     //     //         var rotOffset = bestTarget.get().getYaw();
    //     //     //         rotOffset = targetYawPID.calculate(rotOffset);
    //     //     //         result = Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotOffset)) ) ;
    //     //     //         result = Optional.empty() ;

    //     //     //         // Logger.recordOutput("YawOverrideAlign/targetYaw", bestTarget.get().getYaw());
    //     //     //         // Logger.recordOutput("YawOverrideAlign/proposed rot", result.get());
    //     //     //         // Logger.recordOutput("YawOverrideAlign/rot offset", result.get());
    //     //     //     } else {

    //     //     //     }
    //     //     //     noteYawPID.reset();

    //     //     }
    //     }

    //     return result;
    // }



    public Command getAutonomousCommand() {
        if (autoChooser != null) {
            return autoChooser.getSelected();
        }
        return null;
    }




    public Command getVisionPathPlannerPathing(boolean usingVision, boolean usingOdometryUpdate) {
        Pose2d drivePose = driveSub.getPose();
        if (usingOdometryUpdate) {
            drivePose = driveSub.getPredictedPose();
        }
        double targetX = -1000;
        double targetY = -1000;
        double targetZ = -1000;
        if (usingVision) {
            List<PhotonTrackedTarget> detectedTargets = visionSub.getAllDetectedTargets();

            double minDist = 10000000;
            // int bestID;
            for (PhotonTrackedTarget target : detectedTargets) {
                double d = Math.sqrt((target.getBestCameraToTarget().getX() - drivePose.getX()) + (target.getBestCameraToTarget().getY() - drivePose.getY()));
                // if (d < minDist) {
                    targetX = target.getBestCameraToTarget().getX();
                    targetY = target.getBestCameraToTarget().getY();
                    targetZ = target.getBestCameraToTarget().getZ();
                    // minDist = d;
                    // bestID = target.getFiducialId();
                // }
            }

            if (targetX == -1000 || targetY == -1000 || targetZ == -1000) { // If you don't detect an ID, don't run a path
                // System.out.println("No best trackable");
                return null;
            }
        }
        return setUpPathplannerOTF(usingVision, drivePose, targetX, targetY, targetZ);
    }
    public void changeCoralVision(boolean val) {
        this.moveToCoral = val;
    }
    public void changeToRight(boolean val) { 
        this.goToRight = val;
    }

    public boolean getChangeToRight() {
        return this.goToRight;
    }

    
    public boolean getCoralVision() {
        return this.moveToCoral;
    }

    public Command setUpPathplannerOTF(boolean usingVision, Pose2d drivePose, double targetX, double targetY, double targetZ) {
        Command pathfindingCommand = null;
        double sideOfReef = -1;
        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        if (getChangeToRight()) {
            sideOfReef = 1;
        }
        if (usingVision) {
            double addAmount = 0;
            if (targetZ > 0) { // positive
                addAmount = -180;
            }
            else {
                addAmount = 180;
            }

            Rotation2d reefAutoTargetPose = Rotation2d.fromDegrees(drivePose.getRotation().getDegrees()+(targetZ+addAmount));
            var waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
                new Pose2d(drivePose.getX()+targetX, drivePose.getY()+targetY, reefAutoTargetPose) // vision AprilTag Detection
                // new Pose2d(finalPoseOfAprilTagId.getX()-0.025406 * (Constants.DriveConstants.WheelBase), finalPose?OfAprilTagId.getY()+(Constants.FieldConstants.reefOffsetMeters*sideOfReef), new Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI))
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, reefAutoTargetPose)
            );


            pathfindingCommand = AutoBuilder.followPath(path);

        }
        else {
            Pose3d finalPoseOfAprilTagId = new Pose3d(driveSub.getPose());
            var alliance = DriverStation.getAlliance();
            Logger.recordOutput("alliance", alliance.get());
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Blue) {
                    finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(21).get();
                }
                if (alliance.get() == DriverStation.Alliance.Red) {
                    finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(10).get();
                }

            }
            reefAutoTargetPose = new Pose2d(finalPoseOfAprilTagId.getX()+(Constants.DriveConstants.WheelBase), finalPoseOfAprilTagId.getY()+(Constants.FieldConstants.reefOffsetMeters*sideOfReef), new Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI));
            var waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
                // new Pose2d(drivePose.getX()+targetX, drivePose.getY()+targetY, test2) // vision AprilTag Detection
                reefAutoTargetPose
                // new Pose2d(finalPoseOfAprilTagId.getX()-0.025406 * (Constants.DriveConstants.WheelBase), finalPose?OfAprilTagId.getY()+(Constants.FieldConstants.reefOffsetMeters*sideOfReef), new Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI))
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, reefAutoTargetPose.getRotation())
            );


            pathfindingCommand = AutoBuilder.followPath(path);
        }
        Logger.recordOutput("finalPoseOfTargetAprilTag", reefAutoTargetPose);

        return pathfindingCommand;
    }


}
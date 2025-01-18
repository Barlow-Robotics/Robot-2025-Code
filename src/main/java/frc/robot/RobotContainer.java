
package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

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
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.config.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
// import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* SUBSYSTEMS */
    private static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger moveToSpeakerButton; // button x
    private Trigger moveToAmpButton; // button y
    private Trigger moveToSourceButton; // left stick
    private Trigger moveToFloorButton; // left bumper
    private Trigger moveToFerryButton; // hamburger

    private Trigger climbButton; // button a
    private Trigger piviotToPoint;
    // private Trigger climbAbortButton; // right stick

    // private Trigger toggleLEDsButton; // hamburger
    // private Trigger LEDHumanSourceButton;
    // private Trigger LEDHumanFloorButton;

    private Trigger shootIntakeButton; // trigger
    private Trigger reverseFloorIntakeButton; // driver button 7

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9

    private PIDController noteYawPID;
    private PIDController targetYawPID;

    public final Drive driveSub;
    public final Vision visionSub;
    /* AUTO */

    private SendableChooser<Command> autoChooser;
    boolean moveToCoral;

    public RobotContainer() {
        moveToCoral = false; 
        driveSub = new Drive();
        visionSub = new Vision();
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
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider), true));
    }

    private void configureBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** DRIVE *****************/

        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        restartGyroButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

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
                    if (alliance.isPresent()) {
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
        // autoChooser.setDefaultOption("BASIC", new PathPlannerAuto("BASIC"));
        // autoChooser.addOption("Routine A", new DynamicPathPlanner("Routine A", visionSub));
        // autoChooser.addOption("Routine B", new DynamicPathPlanner("Routine B", visionSub));
        // autoChooser.addOption("Routine C", new DynamicChoreo("Routine C", visionSub, driveSub));
        // autoChooser.addOption("Routine D", new DynamicChoreoCommand("Routine D", visionSub, driveSub));
        

        // autoChooser.addOption("Choreo", driveSub.ChoreoAuto("CompletePath"));
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




    public Command getVisionPathPlannerPathing() {
        List<PhotonTrackedTarget> detectedTargets = visionSub.getAllDetectedTargets();
        double minDist = 10000000;
        int bestID;
        double targetX = -1000;
        double targetY = -1000;
        double targetZ = -1000;
        Pose2d drivePose = driveSub.getPose();
        for (PhotonTrackedTarget target : detectedTargets) {
            double d = Math.sqrt((target.getBestCameraToTarget().getX() - drivePose.getX()) + (target.getBestCameraToTarget().getY() - drivePose.getY()));
            System.out.println("Fiducial ID: " + target.getFiducialId());
            System.out.println("Target X: " + target.getBestCameraToTarget().getX());
            System.out.println("Target Y: " + target.getBestCameraToTarget().getY());
            System.out.println("Target Z: " + target.getBestCameraToTarget().getZ());
            if (d < minDist) {
                targetX = target.getBestCameraToTarget().getX();
                targetY = target.getBestCameraToTarget().getY();
                targetZ = target.getBestCameraToTarget().getZ();
                minDist = d;
                bestID = target.getFiducialId();
            }
        }
        if (targetX == -1000 || targetY == -1000 || targetZ == -1000) { // If you don't detect an ID, don't run a path
            return null;
        }
        return setUpPathplannerOTF(drivePose, targetX, targetY, targetZ);
    }
    public void changeCoralVision(boolean val) {
        this.moveToCoral = val;
    }
    public boolean getCoralVision() {
        return this.moveToCoral;
    }

    public Command setUpPathplannerOTF(Pose2d drivePose, double targetX, double targetY, double targetZ) {
        double addAmount;
        if (targetZ > 0) { // positive
            addAmount = -180;
        }
        else {
            addAmount = 180;
        }
        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
            new Pose2d(drivePose.getX()+targetX, drivePose.getY()+targetY, Rotation2d.fromDegrees(drivePose.getRotation().getDegrees()+(targetZ+addAmount)))
        );
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, Rotation2d.fromDegrees(drivePose.getRotation().getDegrees()+(targetZ+addAmount)))
        );

        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);
        return pathfindingCommand;
        // path.preventFlipping = true;
    }

}
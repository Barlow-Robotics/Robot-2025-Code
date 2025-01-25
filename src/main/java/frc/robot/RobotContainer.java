
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
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.autonomous.DynamicChoreoCommand;
import frc.robot.autonomous.DynamicChoreo;
import frc.robot.autonomous.DynamicPathPlanner;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.SetArmPosition;
import javax.swing.*;
import java.awt.Point;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.ObjectInputFilter.Config;
import java.util.ArrayList;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    /* SUBSYSTEMS */
public Drive driveSub = new Drive();
public Vision visionSub = new Vision();
public final Arm armSub = new Arm(visionSub, driveSub);
public final Climb climbSub = new Climb();
public final CoralIntake coralIntakeSub = new CoralIntake();
public final AlgaeIntake algaeIntakeSub = new AlgaeIntake(visionSub, driveSub);


    /* COMMANDS */
private final SetArmPosition setArmPosHomeCmd = new SetArmPosition(armSub, ArmState.Home);
private final SetArmPosition setArmPosLoadCoralCmd = new SetArmPosition(armSub, ArmState.LoadCoral);
private final SetArmPosition setArmPosLevel4Cmd = new SetArmPosition(armSub, ArmState.Level4);
private final SetArmPosition setArmPosLevel3Cmd = new SetArmPosition(armSub, ArmState.Level3);
private final SetArmPosition setArmPosLevel2Cmd = new SetArmPosition(armSub, ArmState.Level2);
private final SetArmPosition setArmPosLevel1Cmd = new SetArmPosition(armSub, ArmState.Level1);
private final SetArmPosition setArmPosAlgaeLowCmd = new SetArmPosition(armSub, ArmState.AlgaeLow);
private final SetArmPosition setArmPosAlgaeHighCmd = new SetArmPosition(armSub, ArmState.AlgaeHigh);


    /* CONTROLLERS */
    private static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger thisIsAButton; // button x


    private Trigger moveToCoralButton;

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9

    public Trigger moveToHomeButton;
    public Trigger moveToLoadCoralButton;
    public Trigger moveToLevel1Button;
    public Trigger moveToLevel2Button;
    public Trigger moveToLevel3Button;
    public Trigger moveToLevel4Button;
    public Trigger moveToAlgaeHighButton;
    public Trigger moveToAlgaeLowButton;
    
    

    private PIDController noteYawPID;
    private PIDController targetYawPID;

  
   

    
    /* AUTO */

    private SendableChooser<Command> autoChooser;
    boolean moveToCoral;
    // private final RobotCommunicator communicator; 
    // private RobotController robotController;
    public RobotContainer() {
        // communicator = new RobotCommunicator(); // Initialize GUI on the Swing Event Dispatch Thread 
        // SwingUtilities.invokeLater(() -> { robotController = new RobotController(communicator); 
        // SwingUtilities.invokeLater(() -> { 
        //     robotController = new RobotController(communicator); 
        //     JFrame frame = new JFrame("Robot Controller"); 
        //     frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); 
        //     frame.add(robotController); frame.pack(); frame.setVisible(true); 
        // });
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

        /***************** POSITION *****************/
        
        moveToHomeButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToHomeButton.onTrue(setArmPosHomeCmd);
       
        moveToLoadCoralButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToLoadCoralButton.onTrue(setArmPosLoadCoralCmd);

        moveToAlgaeHighButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToAlgaeHighButton.onTrue(setArmPosAlgaeHighCmd);
       
        moveToAlgaeLowButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToHomeButton.onTrue(setArmPosAlgaeLowCmd);

        moveToLevel1Button = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToLevel1Button.onTrue(setArmPosLevel1Cmd);
        
        moveToLevel2Button = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToLevel2Button.onTrue(setArmPosLevel2Cmd);

        moveToLevel3Button = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToLevel3Button.onTrue(setArmPosLevel3Cmd);

        moveToLevel4Button = new JoystickButton(operatorController, XboxControllerConstants.RightBumper); // middle 
        moveToLevel4Button.onTrue(setArmPosLevel4Cmd);
        
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
        System.out.println(detectedTargets);
        // System.out.println(detectedTargets.);

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
            System.out.println("No best trackable");
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
        double addAmount = 0;
        // if (targetZ > 0) { // positive
        //     addAmount = -180;
        // }
        // else {
        //     addAmount = 180;
        // }
        System.out.println("TEST");
        System.out.println(drivePose.getRotation());
        System.out.println(drivePose.getRotation().getDegrees());
        Rotation2d test = new Rotation2d(Math.toRadians(drivePose.getRotation().getDegrees()+(targetZ+addAmount)));
        Rotation2d test2 = Rotation2d.fromDegrees(drivePose.getRotation().getDegrees()+(targetZ+addAmount));
        Rotation2d finalRotation = Rotation2d.fromDegrees(
            MathUtil.inputModulus(drivePose.getRotation().getDegrees() + (targetZ + addAmount), 0, 360)
        );

        System.out.println(test.getDegrees());
        var waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
            // new Pose2d(drivePose.getX()+1, drivePose.getY(), drivePose.getRotation())

            new Pose2d(drivePose.getX()+targetX, drivePose.getY()+targetY, test2)
        );

        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        System.out.println("TEST2");

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, test2)
        );

        System.out.println("TES3T");

        Command pathfindingCommand = AutoBuilder.followPath(path);
        return pathfindingCommand;
    }

    // public Command test() {
    //     List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //     new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
    //     new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    //     );

    //     PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    //     // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    //     // Create the path using the waypoints created above
    //     PathPlannerPath path = new PathPlannerPath(
    //             waypoints,
    //             constraints,
    //             null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
    //             new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    //     );

    //     // Prevent the path from being flipped if the coordinates are already correct
    //     path.preventFlipping = true;

    // }

}

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
// import com.pathplanner.lib.config.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;

import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.LogitechExtreme3DConstants;
// import frc.robot.commands.DriveRobot;
import frc.robot.commands.RunGripper;
import frc.robot.commands.SetArmPosition;
import frc.robot.commands.StopAlgaeIntake;
import frc.robot.commands.StopGripper;
import frc.robot.commands.DeliverCoral;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.RemoveAlgae;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* SUBSYSTEMS */
    public Drive driveSub = TunerConstants.createDrivetrain();
    public Vision visionSub = new Vision(driveSub);
    public final Gripper gripperSub = new Gripper();
    public final Arm armSub = new Arm(visionSub, driveSub, gripperSub);
    public final Climb climbSub = new Climb();
    public final AlgaeIntake algaeIntakeSub = new AlgaeIntake();

    /* COMMANDS */
    // private final SetArmPosition setArmPosHomeCmd = new SetArmPosition(armSub,
    // ArmState.Home);

    private final SetArmPosition setArmPosLoadCoralCmd = new SetArmPosition(armSub, ArmState.LoadCoral);
    private final SetArmPosition setArmPosLevel4Cmd = new SetArmPosition(armSub, ArmState.Level4);
    private final SetArmPosition setArmPosLevel3Cmd = new SetArmPosition(armSub, ArmState.Level3);
    private final SetArmPosition setArmPosLevel2Cmd = new SetArmPosition(armSub, ArmState.Level2);
    private final SetArmPosition setArmPosLevel1Cmd = new SetArmPosition(armSub, ArmState.Level1);
    private final SetArmPosition setArmPosAlgaeLowCmd = new SetArmPosition(armSub, ArmState.AlgaeLow);
    private final SetArmPosition setArmPosAlgaeHighCmd = new SetArmPosition(armSub, ArmState.AlgaeHigh);
    private final SetArmPosition setArmPosAlageEndCmd = new SetArmPosition(armSub, ArmState.FinishRemovingAlgae);

    private final EjectAlgae ejectAlgaeCmd = new EjectAlgae(algaeIntakeSub);
    private final IntakeAlgae intakeAlgaeCmd = new IntakeAlgae(algaeIntakeSub);
    private final StopAlgaeIntake stopAlgaeIntakeCmd = new StopAlgaeIntake(algaeIntakeSub);

    private final RunGripper runGripperCmd = new RunGripper(gripperSub, armSub);
    private final StopGripper stopGripperCmd = new StopGripper(gripperSub);

    private final DeliverCoral deliverCoralCmd = new DeliverCoral(armSub, gripperSub);
    private final RemoveAlgae removeAlgaeCmd = new RemoveAlgae(setArmPosAlageEndCmd, runGripperCmd,
            setArmPosLoadCoralCmd);

    /* CONTROLLERS */
    /* private */ static Joystick driverController;
    /* private */ static Joystick operatorController;

    /* BUTTONS */
    private Trigger resetFieldRelativeButton;

    private Trigger autoAlignRightButton;
    private Trigger autoAlignLeftButton;
    private Trigger resetOdometryToVision;

    // private Trigger climbAbortButton; // right stick

    // private Trigger toggleLEDsButton; // hamburger
    // private Trigger LEDHumanSourceButton;
    // private Trigger LEDHumanFloorButton;

    private Trigger shootIntakeButton; // trigger
    private Trigger reverseFloorIntakeButton; // driver button 7

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9

    // private Trigger moveToHomeButton;
    private Trigger moveToLoadCoralButton;
    private Trigger moveToLevel1Button;
    private Trigger moveToLevel2Button;
    private Trigger moveToLevel3Button;
    private Trigger moveToLevel4Button;
    private Trigger moveToAlgaeHighButton;
    private Trigger moveToAlgaeLowButton;

    private Trigger intakeAlgaeButton;
    private Trigger ejectAlgaeButton;
    private Trigger retractIntakeButton;


    private Trigger autoAlignAlgaeButton;
    private Trigger autoAlignAlgaeButton_2;


    private Trigger runGripperButton;
    private Trigger ejectCoralButton;
    private Trigger shooterButton;

    /* PID */
    private PIDController noteYawPID;
    private PIDController targetYawPID;

    /* AUTO */
    private SendableChooser<Command> autoChooser;
    boolean moveToCoral;
    Boolean goToRight;

    public Pose2d reefAutoTargetPose = new Pose2d();

    /* DRIVE STUFF */

    private final LinearFilter xVelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter yVelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter twistFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxDriveableVelocity * 0.1)
            .withRotationalDeadband(Units.radiansToRotations(DriveConstants.MaxAngularRadiansPerSecond) * 0.1) // Add a
                                                                                                               // 10%
                                                                                                               // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public RobotContainer() {

        goToRight = false;
        // communicator = new RobotCommunicator(); // Initialize GUI on the Swing Event
        // Dispatch Thread
        // SwingUtilities.invokeLater(() -> { robotController = new
        // RobotController(communicator);
        // SwingUtilities.invokeLater(() -> {
        // robotController = new RobotController(communicator);
        // JFrame frame = new JFrame("Robot Controller");
        // frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        // frame.add(robotController); frame.pack(); frame.setVisible(true);
        // });
        moveToCoral = false;
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
                // Drivetrain will execute this command periodically
                driveSub.applyRequest(() -> {
                    double xVelInput = -driverController.getY();
                    double yVelInput = -driverController.getX();
                    double twistInput = -driverController.getTwist();
                    Logger.recordOutput("Drive/xVelInput", xVelInput);
                    Logger.recordOutput("Drive/yVelInput", yVelInput);
                    Logger.recordOutput("Drive/twistInput", twistInput);

                    double xVelFiltered = xVelFilter.calculate(xVelInput);
                    double yVelFiltered = yVelFilter.calculate(yVelInput);
                    double twistFiltered = twistFilter.calculate(twistInput);

                    Logger.recordOutput("Drive/xVelFiltered", xVelFiltered);
                    Logger.recordOutput("Drive/yVelFiltered", yVelFiltered);
                    Logger.recordOutput("Drive/twistFiltered", twistFiltered);

                    double xVelConditioned = xVelFiltered * xVelFiltered * Math.signum(xVelFiltered);
                    double yVelConditioned = yVelFiltered * yVelFiltered * Math.signum(yVelFiltered);
                    double twistConditioned = twistFiltered * twistFiltered * Math.signum(twistFiltered);

                    double velocityMag = Math
                            .sqrt(xVelConditioned * xVelConditioned + yVelConditioned * yVelConditioned);
                    if (velocityMag > 1) {
                        xVelConditioned /= velocityMag;
                        yVelConditioned /= velocityMag;
                    }

                    Logger.recordOutput("Drive/xVelConditioned", xVelConditioned);
                    Logger.recordOutput("Drive/yVelConditioned", yVelConditioned);
                    Logger.recordOutput("Drive/twistConditioned", twistConditioned);

                    return drive.withVelocityX(xVelConditioned * DriveConstants.MaxDriveableVelocity) // Drive forward
                                                                                                      // with negative Y
                                                                                                      // (forward)
                            .withVelocityY(yVelConditioned * DriveConstants.MaxDriveableVelocity) // Drive left with
                                                                                                  // negative X (left)
                            .withRotationalRate(twistConditioned * DriveConstants.MaxAngularRadiansPerSecond); // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                }));
        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configurePathPlanner();
    }

    // public void stop() {
    //     motor.set(0);
    // }
 
    public void disableSubsytems() {
        driveSub.stopDrive();
        armSub.stopArmMotor();
        armSub.stopWristMotor();
        armSub.stopCarriageMotor();
        armSub.stopElevatorMotor();
        climbSub.stop();
        gripperSub.stop();
        algaeIntakeSub.stopIntakeMotor();
        algaeIntakeSub.stopLiftMotor();
    }

    private void configureBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** DRIVE *****************/

        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        // restartGyroButton = new JoystickButton(driverController,
        // LogitechExtreme3DConstants.Button9);
        // restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

        // reset the field-centric heading on left bumper press
        resetFieldRelativeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        resetFieldRelativeButton.onTrue(driveSub.runOnce(() -> driveSub.seedFieldCentric()));

        // moveToCoralButton = new JoystickButton(driverController,
        // LogitechExtreme3DConstants.Button8);
        // moveToCoralButton.onTrue(new InstantCommand(() -> changeCoralVision(true)))
        // .onFalse(new InstantCommand(() -> changeCoralVision(false)));

        /***************** POSITION *****************/

        moveToLoadCoralButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonY); // CHANGE
        moveToLoadCoralButton.onTrue(setArmPosLoadCoralCmd);

        moveToAlgaeHighButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonA); // CHANGE
        moveToAlgaeHighButton.onTrue(setArmPosAlgaeHighCmd);

        moveToAlgaeLowButton = new JoystickButton(operatorController, LogitechDAConstants.ButtonB); // CHANGE
        moveToAlgaeLowButton.onTrue(setArmPosAlgaeLowCmd);

        moveToLevel1Button = new JoystickButton(operatorController, LogitechDAConstants.RightBumper); // CHANGE
        moveToLevel1Button.onTrue(setArmPosLevel1Cmd);

        moveToLevel2Button = new JoystickButton(operatorController, LogitechDAConstants.LeftBumper); // CHANGE
        moveToLevel2Button.onTrue(setArmPosLevel2Cmd);

        moveToLevel3Button = new JoystickButton(operatorController, LogitechDAConstants.ButtonX); // CHANGE
        moveToLevel3Button.onTrue(setArmPosLevel3Cmd);

        moveToLevel4Button = new JoystickButton(operatorController, LogitechDAConstants.LeftStick); // CHANGE
        moveToLevel4Button.onTrue(setArmPosLevel4Cmd);

        /***************** ALGAE INTAKE *****************/

        intakeAlgaeButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); // CHANGE
        intakeAlgaeButton.onTrue(intakeAlgaeCmd).onFalse(stopAlgaeIntakeCmd);

        ejectAlgaeButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); // CHANGE
        ejectAlgaeButton.onTrue(ejectAlgaeCmd).onFalse(stopAlgaeIntakeCmd);

        retractIntakeButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); // CHANGE
        retractIntakeButton.onTrue(stopAlgaeIntakeCmd);

        /***************** GRIPPER *****************/
        
        // runGripperButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); // CHANGE
        // runGripperButton.onTrue(Commands.parallel(runGripperCmd.andThen(stopGripperCmd), setArmPosLoadCoralCmd)).onFalse(stopGripperCmd);

        shooterButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Trigger);
        shooterButton.onTrue(deliverCoralCmd);

        autoAlignAlgaeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button5);
        autoAlignAlgaeButton.onTrue(new InstantCommand(() -> changeToLeft(null)))
                .onFalse(new InstantCommand(() -> disableCoralVision()));

        autoAlignAlgaeButton_2 = new JoystickButton(driverController, LogitechExtreme3DConstants.Button6);
        autoAlignAlgaeButton_2.onTrue(new InstantCommand(() -> changeToLeft(null)))
                .onFalse(new InstantCommand(() -> disableCoralVision()));

        autoAlignRightButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button3);
        autoAlignRightButton.onTrue(new InstantCommand(() -> changeToLeft(true)))
                .onFalse(new InstantCommand(() -> disableCoralVision()));
        
        autoAlignLeftButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button4);
        autoAlignLeftButton.onTrue(new InstantCommand(() -> changeToLeft(false)))
                .onFalse(new InstantCommand(() -> disableCoralVision()));

        resetOdometryToVision = new JoystickButton(driverController, LogitechExtreme3DConstants.Button10);
        resetOdometryToVision.onTrue(new InstantCommand(() -> driveSub.resetPose(driveSub.getPose())));
    }

    public void configurePathPlanner() {

        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
        }

        /* PATHPLANNER INIT */
        NamedCommands.registerCommand("setPositionCoralL1", setArmPosLevel1Cmd);
        NamedCommands.registerCommand("setPositionCoralL2", setArmPosLevel2Cmd);
        NamedCommands.registerCommand("setPositionCoralL3", setArmPosLevel3Cmd);
        NamedCommands.registerCommand("setPositionCoralL4", setArmPosLevel4Cmd);
        NamedCommands.registerCommand("startOuttake", deliverCoralCmd);
        // NamedCommands.registerCommand("setPositionCoralL4", );
        // NamedCommands.registerCommand("setPositionCoralL4", );
        // NamedCommands.registerCommand("setPositionCoralL4", );

        // NamedCommands.registerCommand("StopShooterIntake", stopShooterIntakeCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionAmp",
        // setShooterPosAmpCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionSpeaker",
        // setShooterPosSpeakerCmd);
        // NamedCommands.registerCommand("SetShooterMountPositionFloor",
        // setShooterPosFloorCmd);
        // NamedCommands.registerCommand("start_intake_shoot", new
        // SequentialCommandGroup(setShooterPosSpeakerCmd, startShooterIntakeCmd));

        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        // if (alliance.get() == DriverStation.Alliance.Red) {
        // NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommandRED);
        // NamedCommands.registerCommand("PivotToCenterNote",
        // piviotToCenterNoteCommandRED);
        // NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommandRED);
        // NamedCommands.registerCommand("PivotToStageNote",
        // piviotToStageNoteCommandRED);
        // } else {
        // NamedCommands.registerCommand("PivotToSpeaker", piviotToSpeakerCommand);
        // NamedCommands.registerCommand("PivotToCenterNote",
        // piviotToCenterNoteCommand);
        // NamedCommands.registerCommand("PivotToAmpNote", piviotToAmpNoteCommand);
        // NamedCommands.registerCommand("PivotToStageNote", piviotToStageNoteCommand);
        // }
        // }

        autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio and
                                                      // delete them there
        SmartDashboard.putData("Selected Auto", autoChooser);
        autoChooser.setDefaultOption("Choreo 1Auto", driveSub.ChoreoAuto("Top Left 1 Coral"));
        // autoChooser.addOption("Routine A", new DynamicPathPlanner("Routine A",
        // visionSub));
        // autoChooser.addOption("Routine B", new DynamicPathPlanner("Routine B",
        // visionSub));
        // autoChooser.addOption("Routine C", new DynamicChoreo("Routine C", visionSub,
        // driveSub));
        // autoChooser.addOption("Routine D", new DynamicChoreoCommand("Routine D",
        // visionSub, driveSub));
        autoChooser.addOption("VisionTest", new PathPlannerAuto("TestVision"));

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

        // Logger.recordOutput(, nu);
    }

    // public Optional<Rotation2d> getRotationTargetOverride(){

    // Optional<Rotation2d> result = Optional.empty() ;

    // // if (RobotState.isAutonomous()) {

    // // // if ( shooterMountSub.getShooterMountState() ==
    // ShooterMountState.FloorIntake ) {
    // // // if (visionSub.noteIsVisible() &&
    // Math.abs(visionSub.getNoteDistanceFromCenter()) <
    // Constants.VisionConstants.NoteAlignPixelTolerance) {
    // // // double yaw = visionSub.getNoteDistanceFromCenter();
    // // // double rotDelta = noteYawPID.calculate(yaw);
    // // // result = Optional.of( driveSub.getPose().getRotation().plus(new
    // Rotation2d(rotDelta)) ) ;
    // // // } else {
    // // // noteYawPID.reset();
    // // // }
    // // // } else if (shooterMountSub.getShooterMountState() ==
    // ShooterMountState.Speaker && !visionSub.allDetectedTargets.isEmpty()) {
    // // // var bestTarget = visionSub.getBestTrackableTarget() ;
    // // // if (bestTarget.isPresent()) {
    // // // var rotOffset = bestTarget.get().getYaw();
    // // // rotOffset = targetYawPID.calculate(rotOffset);
    // // // result = Optional.of( driveSub.getPose().getRotation().plus(new
    // Rotation2d(rotOffset)) ) ;
    // // // result = Optional.empty() ;

    // // // // Logger.recordOutput("YawOverrideAlign/targetYaw",
    // bestTarget.get().getYaw());
    // // // // Logger.recordOutput("YawOverrideAlign/proposed rot", result.get());
    // // // // Logger.recordOutput("YawOverrideAlign/rot offset", result.get());
    // // // } else {

    // // // }
    // // // noteYawPID.reset();

    // // }
    // }

    // return result;
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
            drivePose = driveSub.getPose();
            driveSub.resetPose(drivePose);
        }

        double targetX = -1000;
        double targetY = -1000;
        double targetZ = -1000;
        if (usingVision) {
            var detectedTargets = visionSub.getAllDetectedTargets();

            double minDist = 10000000;
            // int bestID;
            for (PhotonTrackedTarget target : detectedTargets) {
                double d = Math.sqrt((target.getBestCameraToTarget().getX() - drivePose.getX())
                        + (target.getBestCameraToTarget().getY() - drivePose.getY()));
                // if (d < minDist) {
                targetX = target.getBestCameraToTarget().getX();
                targetY = target.getBestCameraToTarget().getY();
                targetZ = target.getBestCameraToTarget().getZ();
                // minDist = d;
                // bestID = target.getFiducialId();
                // }
            }

            if (targetX == -1000 || targetY == -1000 || targetZ == -1000) { // If you don't detect an ID, don't run a
                                                                            // path
                // System.out.println("No best trackable");
                return null;
            }
        }
        return setUpPathplannerOTF(usingVision, drivePose, targetX, targetY, targetZ);
    }

    // public void changeCoralVision(boolean val) {
    // this.moveToCoral = val;
    // }
    public void changeToLeft(Boolean val) {
        this.goToRight = val;
        this.moveToCoral = true;
    }

    public void disableCoralVision() {
        this.moveToCoral = false;
    }

    public Boolean getChangeToRight() {
        return this.goToRight;
    }

    public boolean getCoralVision() {
        return this.moveToCoral;
    }

    public int findClosestToRobot(Pose2d drivePose, int[] aprilTagList) {
        double minValue = Integer.MAX_VALUE;
        int idMinValue = -1;
        for (int i = 0; i < aprilTagList.length; i++) {
            Pose3d poseAprilTag = visionSub.getLayout().getTagPose(aprilTagList[i]).get();
            double distance = Math.abs(drivePose.getX() - poseAprilTag.getX())
                    + Math.abs(drivePose.getY() - poseAprilTag.getY());
            if (distance < minValue) {
                minValue = distance;
                idMinValue = aprilTagList[i];
            }
        }
        return idMinValue;
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> driveSub.getState().Pose, // Supplier of current robot pose
                    driveSub::resetPose, // Consumer for seeding pose against auto
                    () -> driveSub.getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> driveSub.setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent() && DriverStation.isAutonomous()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }

                        return false;
                    },

                    driveSub // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    public Command setUpPathplannerOTF(boolean usingVision, Pose2d drivePose, double targetX, double targetY,
            double targetZ) {
        Command pathfindingCommand = null;
        double sideOfReef = -1;
        PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for
        if (getChangeToRight() == null) {
            sideOfReef = 0;
        }
        else if (getChangeToRight()) {
            sideOfReef = 1;
        }
        if (usingVision) {
            double addAmount = 0;
            if (targetZ > 0) { // positive
                addAmount = -180;
            } else {
                addAmount = 180;
            }

            Rotation2d reefAutoTargetPose = Rotation2d
                    .fromDegrees(drivePose.getRotation().getDegrees() + (targetZ + addAmount));
            var waypoints = PathPlannerPath.waypointsFromPoses(
                    new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
                    new Pose2d(drivePose.getX() + targetX, drivePose.getY() + targetY, reefAutoTargetPose) // vision
                                                                                                           // AprilTag
                                                                                                           // Detection
            // new Pose2d(finalPoseOfAprilTagId.getX()-0.025406 *
            // (Constants.DriveConstants.WheelBase),
            // finalPose?OfAprilTagId.getY()+(Constants.FieldConstants.reefOffsetMeters*sideOfReef),
            // new
            // Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI))
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, reefAutoTargetPose));

            pathfindingCommand = AutoBuilder.followPath(path);

        } else {
            Pose3d finalPoseOfAprilTagId = new Pose3d(driveSub.getPose());
            var alliance = DriverStation.getAlliance();
            Logger.recordOutput("alliance", alliance.get());
            if (alliance.isPresent()) {
                if (armSub.isAvailableToGoToReef()) {
                    if (alliance.get() == DriverStation.Alliance.Blue) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.blueAprilTagListReef);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.redAprilTagListReef);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                }
                if (armSub.isAvailableToGoToCoralStation()) {
                    if (alliance.get() == DriverStation.Alliance.Blue) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.blueAprilTagListCoralStation);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.redAprilTagListCoralStation);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                }


            }
            // cosine of the degree is multiplied by side of reef.
            double radianRobot = finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians();
            // System.out.println(radianRobot);
            double offsetX = Constants.DriveConstants.distanceToFrontOfRobot * Math.cos(radianRobot);
            double offsetY = Constants.DriveConstants.distanceToFrontOfRobot * Math.sin(radianRobot);
            double reefX = finalPoseOfAprilTagId.getX() + offsetX + Constants.FieldConstants.reefOffsetMeters * (sideOfReef * Math.sin(radianRobot));
            double reefY = finalPoseOfAprilTagId.getY() + offsetY + Constants.FieldConstants.reefOffsetMeters * (sideOfReef * -Math.cos(radianRobot));
            reefAutoTargetPose = new Pose2d(reefX, reefY, new Rotation2d(radianRobot + Math.PI));
            // reefAutoTargetPose = new Pose2d(finalPoseOfAprilTagId.getX()
            //         + Constants.DriveConstants.distanceToFrontOfRobot*Math.cos(radianRobot) + Constants.FieldConstants.reefOffsetMeters
            //         * (sideOfReef * Math.sin(radianRobot)),
            //         finalPoseOfAprilTagId.getY() + (Constants.FieldConstants.reefOffsetMeters
            //                 * (sideOfReef * Math.cos(radianRobot))) + Constants.DriveConstants.distanceToFrontOfRobot*Math.sin(radianRobot),
            //         new Rotation2d(radianRobot + Math.PI));
            var waypoints = PathPlannerPath.waypointsFromPoses(
                    new Pose2d(drivePose.getX(), drivePose.getY(), drivePose.getRotation()),
                    // new Pose2d(drivePose.getX()+targetX, drivePose.getY()+targetY, test2) //
                    // vision AprilTag Detection
                    reefAutoTargetPose
            // new Pose2d(finalPoseOfAprilTagId.getX()-0.025406 *
            // (Constants.DriveConstants.WheelBase),
            // finalPose?OfAprilTagId.getY()+(Constants.FieldConstants.reefOffsetMeters*sideOfReef),
            // new
            // Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI))
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, reefAutoTargetPose.getRotation()));

            pathfindingCommand = AutoBuilder.followPath(path);
        }
        Logger.recordOutput("finalPoseOfTargetAprilTag", reefAutoTargetPose);

        return pathfindingCommand;
    }

}
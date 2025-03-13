
package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.LogitechExtreme3DConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.ArmStateManager;
import frc.robot.commands.DoClimb;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.LoadCoralFromChute;
import frc.robot.commands.PositionGripper;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.StopAlgaeIntake;
import frc.robot.commands.LockWheels;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
/* SUBSYSTEMS */
public Drive driveSub = TunerConstants.createDrivetrain();
public final Gripper gripperSub = new Gripper();
public final Elevator elevatorSub;
public final Vision visionSub;
public final Arm armSub;
public final Wrist wristSub;
public final Climb climbSub ;
public final AlgaeIntake algaeIntakeSub = new AlgaeIntake();
public final ArmStateManager armState = new ArmStateManager();

/* COMMANDS */
// private final SetArmPosition setArmPosHomeCmd = new SetArmPosition(armSub,
// ArmState.Home);
private final PositionGripper setArmPosTravellingCmd;
// private final SetArmPosition setArmPosTravellingCmd;
private final LoadCoralFromChute setArmPosLoadCoralCmd;
private final PositionGripper setArmPosLevel4Cmd;
private final PositionGripper setArmPosLevel3Cmd;
private final PositionGripper setArmPosLevel2Cmd;
private final PositionGripper setArmPosLevel1Cmd;

// CHANGE - need to make sure this is right
private final PositionGripper setArmPosAlgaeCmd;
// private final PositionGripper setArmPosAlageEndCmd;

private final EjectAlgae ejectAlgaeCmd;
private final IntakeAlgae intakeAlgaeCmd;
private final StopAlgaeIntake stopAlgaeIntakeCmd;

// private final StartIntakingWithGripper runGripperCmd;
// private final StopGripper stopGripperCmd;

private final ScoreCoral scoreCoralCmd;
private final RemoveAlgae removeAlgaeCmd;

private final DoClimb startClimbingCmd;
private final LockWheels lockWheelsCmd;

/* CONTROLLERS */
/* private */ static Joystick driverController;
/* private */ static Joystick operatorController;
private static Joystick testController ;

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

// private Trigger moveToHomeButton;
private Trigger moveToTravellingButton;
private Trigger moveToLevel1Button;
private Trigger moveToLevel2Button;
private Trigger moveToLevel3Button;
private Trigger moveToLevel4Button;
private Trigger moveToLoadCoralButton ;
// private Trigger moveToAlgaeButton;
private Trigger removeAlgaeButton;

private Trigger startClimbButton;

private Trigger intakeAlgaeButton;
private Trigger scoreAlgaeButton;
private Trigger retractIntakeButton;

private Trigger autoAlignAlgaeButton;
private Trigger autoAlignAlgaeButton_2;

private Trigger runGripperButton;
private Trigger scoreCoralButton;

private POVButton leftPovButton;
private POVButton rightPovButton;
private POVButton upPovButton;
private POVButton downPovButton;

private Trigger lockWheelsButton;

private Trigger disableVisionButton;
private Trigger enableVisionButton;

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
private final LinearFilter sliderFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.MaxDriveableVelocity * 0.1)
        .withRotationalDeadband(Units.radiansToRotations(DriveConstants.MaxAngularRadiansPerSecond) * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
private final SwerveRequest.ApplyRobotSpeeds nudge = new SwerveRequest.ApplyRobotSpeeds();
private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

public RobotContainer(Robot robot) {
    visionSub = new Vision(driveSub, robot);
    elevatorSub = new Elevator(robot);
    armSub = new Arm(robot);
    wristSub = new Wrist(robot);

    climbSub = new Climb( 
        robot ,
        () -> testController.getRawButton(Constants.LogitechDAConstants.ButtonY),
        () -> testController.getPOV() == 270,  // pressing the POV to the left to unwind
        () -> testController.getPOV() == 90    // pressing the POV to the right to wind
        ) ;


    setArmPosTravellingCmd = new PositionGripper(armState, ArmState.Running, elevatorSub, armSub, wristSub);
    setArmPosLoadCoralCmd = new LoadCoralFromChute(elevatorSub, armSub, wristSub, gripperSub, armState);
    setArmPosLevel4Cmd = new PositionGripper(armState, ArmState.Level4, elevatorSub, armSub, wristSub);
    setArmPosLevel3Cmd = new PositionGripper(armState, ArmState.Level3, elevatorSub, armSub, wristSub);
    setArmPosLevel2Cmd = new PositionGripper(armState, ArmState.Level2, elevatorSub, armSub, wristSub);
    setArmPosLevel1Cmd = new PositionGripper(armState, ArmState.Level1, elevatorSub, armSub, wristSub);

    // CHANGE - need to make sure this is right
    setArmPosAlgaeCmd = new PositionGripper(armState, ArmState.StartAlgaePosition, elevatorSub, armSub, wristSub);
    // setArmPosAlageEndCmd = new SetArmPosition(armSub,
    // ArmState.FinishRemovingAlgae);

    ejectAlgaeCmd = new EjectAlgae(algaeIntakeSub);
    intakeAlgaeCmd = new IntakeAlgae(algaeIntakeSub);
    stopAlgaeIntakeCmd = new StopAlgaeIntake(algaeIntakeSub);
    // runGripperCmd = new StartIntakingWithGripper(gripperSub, armSub);
    // stopGripperCmd = new StopGripper(gripperSub);
    scoreCoralCmd = new ScoreCoral(armState, elevatorSub, armSub, wristSub, gripperSub);
    removeAlgaeCmd = new RemoveAlgae(armState, elevatorSub, armSub, wristSub, gripperSub);

    startClimbingCmd = new DoClimb(climbSub, armSub, armState, elevatorSub, wristSub);
    lockWheelsCmd = new LockWheels(driveSub);

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

    drivePovBindings();
    // driveSub.setVisionMeasurementStdDevs(SingleTagStdDevs);
        driveSub.setDefaultCommand(
                // Drivetrain will execute this command periodically
                driveSub.applyRequest(() -> {
                    double xVelInput = -driverController.getY();
                    double yVelInput = -driverController.getX();
                    double twistInput = -driverController.getTwist();
                    double sliderInput = -driverController.getThrottle();
                    
                    Logger.recordOutput("Drive/xVelInput", xVelInput);
                    Logger.recordOutput("Drive/yVelInput", yVelInput);
                    Logger.recordOutput("Drive/twistInput", twistInput);
                    Logger.recordOutput("Drive/sliderInput", sliderInput);

                    double xVelFiltered = xVelFilter.calculate(xVelInput);
                    double yVelFiltered = yVelFilter.calculate(yVelInput);
                    double twistFiltered = twistFilter.calculate(twistInput);

                    Logger.recordOutput("Drive/xVelFiltered", xVelFiltered);
                    Logger.recordOutput("Drive/yVelFiltered", yVelFiltered);
                    Logger.recordOutput("Drive/twistFiltered", twistFiltered);

                    double xVelConditioned = xVelFiltered * xVelFiltered * Math.signum(xVelFiltered);
                    double yVelConditioned = yVelFiltered * yVelFiltered * Math.signum(yVelFiltered);
                    double twistConditioned = twistFiltered * twistFiltered * Math.signum(twistFiltered);
                    
                    // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
                    double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;

                    Logger.recordOutput("Drive/velocityMultiplier", maxVelocityMultiplier);

                    double velocityMag = /* maxVelocityMultiplier * */ Math
                            .sqrt(xVelConditioned * xVelConditioned + yVelConditioned * yVelConditioned);
                    if (velocityMag > 1) {
                        xVelConditioned /= velocityMag;
                        yVelConditioned /= velocityMag;
                    }

                    Logger.recordOutput("Drive/xVelConditioned", xVelConditioned);
                    Logger.recordOutput("Drive/yVelConditioned", yVelConditioned);
                    Logger.recordOutput("Drive/twistConditioned", twistConditioned);

                    Logger.recordOutput("Drive/realVelX", xVelConditioned * DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
                    Logger.recordOutput("Drive/realVelY", yVelConditioned * DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier);
                    Logger.recordOutput("Drive/realVelRot", twistConditioned * DriveConstants.MaxAngularRadiansPerSecond * maxVelocityMultiplier);

                    return drive.withVelocityX(xVelConditioned * DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier)// Drive forward with negative Y (forward)
                                .withVelocityY(yVelConditioned * DriveConstants.MaxDriveableVelocity * maxVelocityMultiplier) // Drive left with negative X (left)
                                .withRotationalRate(twistConditioned * DriveConstants.MaxAngularRadiansPerSecond * maxVelocityMultiplier); // Drive counterclockwise with negative X (left)
                // }) .andThen(driveSub.applyRequest(() -> {
                    
                    // double povInput = driverController.getPOV();

                    // Logger.recordOutput("Drive/povInput", povInput);

                    // double nudgeVelX = 0;
                    // double nudgeVelY = 0;
                    // if (Math.abs(povInput-90.0) > 10) { // within tolerance of right pov
                    //     nudgeVelX = DriveConstants.NudgeSpeed;
                    // } else if (Math.abs(povInput-270.0) >= 10) { // within tolerance of left pov
                    //     nudgeVelX = -DriveConstants.NudgeSpeed;
                    // } else if (Math.abs(povInput-180.0) >= 10) { // within tolerance of down pov
                    //     nudgeVelY = DriveConstants.NudgeSpeed;
                    // } else if (Math.abs(povInput-0.0) >= 10 || Math.abs(povInput-360.0) >= 10) {
                    //     nudgeVelY = -DriveConstants.NudgeSpeed;
                    // }

                    // ChassisSpeeds nudgeSpeeds = new ChassisSpeeds(nudgeVelX, nudgeVelY, 0.0);
                    // return nudge.withSpeeds(nudgeSpeeds);
                }));
        configureAutoBuilder();
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        configurePathPlanner();
    }

    // public void stop() {
    //     motor.set(0);
    // }
 
    public void disableSubsytems() {
        driveSub.stopDrive();
        armSub.stopArmMotor();
        wristSub.stopWristMotor();
        elevatorSub.stopCarriageMotor();
        elevatorSub.stopElevatorMotor();
        climbSub.stop();
        gripperSub.stop();
        algaeIntakeSub.stopIntakeMotor();
        algaeIntakeSub.stopLiftMotor();
    }

    private void configureBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);
        testController = new Joystick(ElectronicsIDs.TestControllerPort) ;

        /***************** DRIVE *****************/

        // reset the field-centric heading on left bumper press
        resetFieldRelativeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        resetFieldRelativeButton.onTrue(driveSub.runOnce(() -> driveSub.seedFieldCentric()));

        lockWheelsButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button4);
        lockWheelsButton.onTrue(lockWheelsCmd);

        // moveToCoralButton = new JoystickButton(driverController,
        // LogitechExtreme3DConstants.Button8);
        // moveToCoralButton.onTrue(new InstantCommand(() -> changeCoralVision(true)))
        // .onFalse(new InstantCommand(() -> changeCoralVision(false)));

        /***************** ARM POSITION *****************/

        moveToTravellingButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick);
        moveToTravellingButton.onTrue(setArmPosTravellingCmd);
        
        moveToLoadCoralButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper);
        moveToLoadCoralButton.onTrue(setArmPosLoadCoralCmd);

        // moveToAlgaeButton = new JoystickButton(operatorController, XboxControllerConstants.LeftTrigger);
        // moveToAlgaeButton.onTrue(setArmPosAlgaeCmd);

        removeAlgaeButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
        removeAlgaeButton.onTrue(removeAlgaeCmd);

        moveToLevel1Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonY); 
        moveToLevel1Button.onTrue(setArmPosLevel1Cmd);

        moveToLevel2Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
        moveToLevel2Button.onTrue(setArmPosLevel2Cmd);

        moveToLevel3Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonB);
        moveToLevel3Button.onTrue(setArmPosLevel3Cmd);

        moveToLevel4Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonA); 
        moveToLevel4Button.onTrue(setArmPosLevel4Cmd);


        // Joystick testingController = new Joystick(0) ;
       startClimbButton = new JoystickButton(operatorController, XboxControllerConstants.HamburgerButton);
        // startClimbButton = new JoystickButton(testingController, LogitechDAConstants.ButtonY);  // just for testing
        startClimbButton.onTrue(startClimbingCmd);

        /***************** ALGAE INTAKE *****************/

        intakeAlgaeButton = new POVButton(operatorController, XboxControllerConstants.POVDown);
        intakeAlgaeButton.onTrue(intakeAlgaeCmd);

        scoreAlgaeButton = new POVButton(operatorController, XboxControllerConstants.POVRight);
        scoreAlgaeButton.onTrue(ejectAlgaeCmd).onFalse(stopAlgaeIntakeCmd);

        retractIntakeButton = new POVButton(operatorController, XboxControllerConstants.POVUp);
        retractIntakeButton.onTrue(stopAlgaeIntakeCmd);

        /***************** GRIPPER *****************/
        
        // runGripperButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); 
        // runGripperButton.onTrue(Commands.parallel(
        //     runGripperCmd.andThen(new StopGripper(gripperSub)),  // Use a fresh instance directly
        //     setArmPosLoadCoralCmd  // Runs independently
        // )).onFalse(stopGripperCmd); // Stop the gripper immediately when released

        // runGripperButton = new JoystickButton(operatorController, LogitechDAConstants.RightStick); 
        // runGripperButton.onTrue(Commands.parallel(
        //     runGripperCmd.andThen(new StopGripper(gripperSub)),  // Use a fresh instance directly
        //     setArmPosLoadCoralCmd  // Runs independently
        // )).onFalse(stopGripperCmd); // Stop the gripper immediately when released


        scoreCoralButton = new JoystickButton(operatorController, XboxControllerConstants.WindowButton);
        // scoreCoralButton.onTrue(scoreCoralCmd);
        scoreCoralButton.onTrue(scoreCoralCmd);

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


        disableVisionButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button12);
        disableVisionButton.onTrue(new InstantCommand(() -> visionSub.disableTheVision(true))).onFalse(Commands.none());

        enableVisionButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);
        enableVisionButton.onTrue(new InstantCommand(() -> visionSub.disableTheVision(false))).onFalse(Commands.none());

    }

    private void drivePovBindings() {
        leftPovButton = new POVButton(driverController, 270);
        leftPovButton.whileTrue(driveSub.applyRequest(() -> {
            double povInput = driverController.getPOV();
            double sliderInput = driverController.getThrottle();
            Logger.recordOutput("Drive/povInput", povInput);
            Logger.recordOutput("Drive/sliderInput", sliderInput);
            // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
            double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
            double vel = DriveConstants.NudgeSpeed / maxVelocityMultiplier; // might need to multiply by -1
            Logger.recordOutput("Drive/nudgeVelocity", vel);
            ChassisSpeeds nudgeSpeeds = new ChassisSpeeds(0.0, vel, 0.0);
            return nudge.withSpeeds(nudgeSpeeds);}));

        rightPovButton = new POVButton(driverController, 90);
        rightPovButton.whileTrue(driveSub.applyRequest(() -> {
            double povInput = driverController.getPOV();
            double sliderInput = driverController.getThrottle();
            Logger.recordOutput("Drive/povInput", povInput);
            Logger.recordOutput("Drive/sliderInput", sliderInput);
            // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
            double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
            double vel = -1 * DriveConstants.NudgeSpeed / maxVelocityMultiplier; // might need to not multiply by -1
            Logger.recordOutput("Drive/nudgeVelocity", vel);
            ChassisSpeeds nudgeSpeeds = new ChassisSpeeds(0.0, vel, 0.0);
            return nudge.withSpeeds(nudgeSpeeds);}));
        
        upPovButton = new POVButton(driverController, 0);
        upPovButton.whileTrue(driveSub.applyRequest(() -> {
            double povInput = driverController.getPOV();
            double sliderInput = driverController.getThrottle();
            Logger.recordOutput("Drive/povInput", povInput);
            Logger.recordOutput("Drive/sliderInput", sliderInput);
            // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
            double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
            double vel = DriveConstants.NudgeSpeed / maxVelocityMultiplier; // might need to multiply by -1
            Logger.recordOutput("Drive/nudgeVelocity", vel);
            ChassisSpeeds nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0); 
            return nudge.withSpeeds(nudgeSpeeds);}));
        
        downPovButton = new POVButton(driverController, 180);
        downPovButton.whileTrue(driveSub.applyRequest(() -> {
            double povInput = driverController.getPOV();
            double sliderInput = driverController.getThrottle();
            Logger.recordOutput("Drive/povInput", povInput);
            Logger.recordOutput("Drive/sliderInput", sliderInput);
            // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8 m/s)
            double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
            double vel = -1 * DriveConstants.NudgeSpeed / maxVelocityMultiplier; // might not need to multiply by -1
            Logger.recordOutput("Drive/nudgeVelocity", vel);
            ChassisSpeeds nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0);
            return nudge.withSpeeds(nudgeSpeeds);}));
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
        NamedCommands.registerCommand("setPositionTraveling", setArmPosTravellingCmd);
        NamedCommands.registerCommand("startOuttake", scoreCoralCmd);


        autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio and
                                                      // delete them there
        SmartDashboard.putData("Selected Auto", autoChooser);

        autoChooser.addOption("Top 3 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Top 3 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Bottom 2 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Bottom 2 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Left from Bottom 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Left from Bottom 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Right 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Right 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Right 1 Coral (Level3)", new DeferredCommand(() -> driveSub.ChoreoAuto("Right 1 Coral (Level3)"), Set.of(driveSub)));
        autoChooser.addOption("Top Left 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Top Left 1 Coral"), Set.of(driveSub)));

        autoChooser.addOption("Bottom Left 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Bottom Left 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Top Right 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Top Right 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Bottom Right 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Bottom Right 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Top 2 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Top 2 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Bottom 3 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Bottom 3 Coral"), Set.of(driveSub)));
        autoChooser.addOption("Left from Top 1 Coral", new DeferredCommand(() -> driveSub.ChoreoAuto("Left from Top 1 Coral"), Set.of(driveSub)));
        autoChooser.addOption("[TEST] Box Auto", new DeferredCommand(() -> driveSub.ChoreoAuto("Box Auto"), Set.of(driveSub)));
        autoChooser.addOption("[TEST] Straight Line Path", new DeferredCommand(() -> driveSub.ChoreoAuto("Straight Line Path"), Set.of(driveSub)));

        // autoChooser.addOption("VisionTest", new PathPlannerAuto("TestVision"));

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
                            new PIDConstants(0.75, 0, 0.1),
                            // PID constants for rotation
                            new PIDConstants(0.5, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> {
                        // var alliance = DriverStation.getAlliance();
                        // if (alliance.isPresent() && DriverStation.isAutonomous()) {
                        //     return alliance.get() == DriverStation.Alliance.Red;
                        // }

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
        PathConstraints constraints = new PathConstraints(1.5,2.0, 6 * Math.PI, 12 * Math.PI); // The constraints for
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
                if (armState.isAvailableToGoToReef()) {
                    if (alliance.get() == DriverStation.Alliance.Blue) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.blueAprilTagListReef);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                    if (alliance.get() == DriverStation.Alliance.Red) {
                        int id = findClosestToRobot(drivePose, Constants.VisionConstants.redAprilTagListReef);
                        finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    }
                }
                if (armState.isAvailableToGoToCoralStation()) {
                    return Commands.none();
                    // if (alliance.get() == DriverStation.Alliance.Blue) {
                    //     int id = findClosestToRobot(drivePose, Constants.VisionConstants.blueAprilTagListCoralStation);
                    //     finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    // }
                    // if (alliance.get() == DriverStation.Alliance.Red) {
                    //     int id = findClosestToRobot(drivePose, Constants.VisionConstants.redAprilTagListCoralStation);
                    //     finalPoseOfAprilTagId = visionSub.getLayout().getTagPose(id).get();
                    // }
                }
            }

            // cosine of the degree is multiplied by side of reef.
            double radianRobot = finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians();
            // System.out.println(radianRobot);
            double offsetX = Constants.DriveConstants.distanceToFrontOfRobot * Math.cos(radianRobot);
            double offsetY = Constants.DriveConstants.distanceToFrontOfRobot * Math.sin(radianRobot);
            double reefX = finalPoseOfAprilTagId.getX() + offsetX + (Constants.FieldConstants.reefOffsetMeters + (Constants.GripperConstants.locationOfGripperToRobotX*-sideOfReef)) * (sideOfReef * Math.sin(radianRobot));
            double reefY = finalPoseOfAprilTagId.getY() + offsetY + (Constants.FieldConstants.reefOffsetMeters + (Constants.GripperConstants.locationOfGripperToRobotX*-sideOfReef)) * (sideOfReef * -Math.cos(radianRobot));
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
            // new Rotation2d(finalPoseOfAprilTagId.getRotation().toRotation2d().getRadians()+Math.PI))
            );

            PathPlannerPath path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0.0, reefAutoTargetPose.getRotation()));
            
            if (path.getAllPathPoints().size() < 2) { // If the path is broken (only 1 point), prevents crashing.
                return Commands.none();
            }

            pathfindingCommand = AutoBuilder.followPath(path);
        }
        Logger.recordOutput("finalPoseOfTargetAprilTag", reefAutoTargetPose);

        return pathfindingCommand;
    }

}
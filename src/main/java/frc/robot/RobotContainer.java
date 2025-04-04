
package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.commands.DynamicAutoBuilder;
import frc.robot.commands.EjectAlgae;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.LoadCoralFromChute;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.PositionGripper;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.RemoveHighAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ScoreCoralWithoutTravel;
import frc.robot.commands.StopAlgaeIntake;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Underglow;
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
        public final Climb climbSub;
        public final Underglow underglowSub;
        public final Chute chuteSub;
        public final AlgaeIntake algaeIntakeSub = new AlgaeIntake();
        public final ArmStateManager armState = new ArmStateManager();
        public final DynamicAutoBuilder dynAutoBuilder;

        private BooleanSupplier chuteHasCoral;
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
        private final ScoreCoral scoreCoralCmd;
        private final ScoreCoralWithoutTravel scoreCoralCmdWithoutTravelling;
        private final RemoveAlgae removeAlgaeCmd;
        private final RemoveHighAlgae removeHighAlgaeCmd;

        private final DoClimb startClimbingCmd;

        /* CONTROLLERS */

        /* private */ static Joystick driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        /* private */ static Joystick operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);
        private static Joystick testController = new Joystick(ElectronicsIDs.TestControllerPort);

        /* BUTTONS */
        private Trigger resetFieldRelativeButton;

        private Trigger autoAlignRightButton;
        private Trigger autoAlignLeftButton;
        private Trigger resetOdometryToVision;

        // private Trigger climbAbortButton; // right stick

        // private Trigger toggleLEDsButton; // hamburger
        // private Trigger LEDHumanSourceButton;
        // private Trigger LEDHumanFloorButton;

        // private Trigger moveToHomeButton;
        private Trigger moveToTravellingButton;
        private Trigger moveToLevel1Button;
        private Trigger moveToLevel2Button;
        private Trigger moveToLevel3Button;
        private Trigger moveToLevel4Button;
        private Trigger moveToLoadCoralButton;

        // private Trigger moveToAlgaeButton;
        private Trigger removeAlgaeButton;
        private Trigger removeHighAlgaeButton;

        private Trigger startClimbButton;

        private Trigger intakeAlgaeButton;
        private Trigger scoreAlgaeButton;
        private Trigger retractIntakeButton;

        private Trigger autoAlignAlgaeButton;
        private Trigger autoAlignAlgaeButton_2;

        private Trigger scoreCoralButton;

        private POVButton leftPovButton;
        private POVButton rightPovButton;
        private POVButton upPovButton;
        private POVButton downPovButton;
        private POVButton brakePovButton;

        private Trigger lockWheelsButton;

        private Trigger disableVisionButton;
        private Trigger enableVisionButton;

        /* PID */
        // private PIDController noteYawPID;
        // private PIDController targetYawPID;

        /* AUTO */
        private SendableChooser<Command> autoChooser;
        boolean moveToCoral;
        Boolean goToRight;

        public Pose2d reefAutoTargetPose = new Pose2d();

        /* DRIVE STUFF */

        private final LinearFilter xVelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        private final LinearFilter yVelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        private final LinearFilter twistFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
        // private final LinearFilter sliderFilter = LinearFilter.singlePoleIIR(0.1,
        // 0.02);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(0.1)
                        .withRotationalDeadband(0.1) // Add a
                                                     // 10%
                                                     // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.ApplyRobotSpeeds nudge = new SwerveRequest.ApplyRobotSpeeds();
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        public RobotContainer(Robot robot) {
                visionSub = new Vision(driveSub, robot);
                elevatorSub = new Elevator(robot);
                armSub = new Arm(robot);
                wristSub = new Wrist(robot);
                chuteSub = new Chute();
                climbSub = new Climb(
                                robot,
                                () -> testController.getRawButton(Constants.LogitechDAConstants.ButtonY),
                                () -> testController.getPOV() == 270, // pressing the POV to the left to unwind
                                () -> testController.getPOV() == 90 // pressing the POV to the right to wind
                );
                chuteHasCoral = () -> chuteSub.hasCoral();

                underglowSub = new Underglow(robot, chuteSub, climbSub);

                dynAutoBuilder = new DynamicAutoBuilder(driveSub, visionSub, driverController);

                setArmPosTravellingCmd = new PositionGripper(armState, ArmState.Running, elevatorSub, armSub, wristSub);
                setArmPosLoadCoralCmd = new LoadCoralFromChute(elevatorSub, armSub, wristSub, gripperSub, armState);
                setArmPosLevel4Cmd = new PositionGripper(armState, ArmState.Level4, elevatorSub, armSub, wristSub);
                setArmPosLevel3Cmd = new PositionGripper(armState, ArmState.Level3, elevatorSub, armSub, wristSub);
                setArmPosLevel2Cmd = new PositionGripper(armState, ArmState.Level2, elevatorSub, armSub, wristSub);
                setArmPosLevel1Cmd = new PositionGripper(armState, ArmState.Level1, elevatorSub, armSub, wristSub);

                // CHANGE - need to make sure this is right
                setArmPosAlgaeCmd = new PositionGripper(armState, ArmState.StartAlgaePosition, elevatorSub, armSub,
                                wristSub);
                // setArmPosAlageEndCmd = new SetArmPosition(armSub,
                // ArmState.FinishRemovingAlgae);

                ejectAlgaeCmd = new EjectAlgae(algaeIntakeSub);
                intakeAlgaeCmd = new IntakeAlgae(algaeIntakeSub);
                stopAlgaeIntakeCmd = new StopAlgaeIntake(algaeIntakeSub);
                // stopGripperCmd = new StopGripper(gripperSub);
                scoreCoralCmd = new ScoreCoral(armState, elevatorSub, armSub, wristSub, gripperSub);
                scoreCoralCmdWithoutTravelling = new ScoreCoralWithoutTravel(armState, elevatorSub, armSub, wristSub,
                                gripperSub);
                removeAlgaeCmd = new RemoveAlgae(armState, elevatorSub, armSub, wristSub, gripperSub);
                removeHighAlgaeCmd = new RemoveHighAlgae(armState, elevatorSub, armSub, wristSub, gripperSub);

                startClimbingCmd = new DoClimb(climbSub, armSub, armState, elevatorSub, wristSub);

                goToRight = false;
                // communicator = new RobotCommunicator(); // Initialize GUI on the Swing Event
                // Dispatch Thread
                // SwingUtilities.invokeLater(() -> { robotController = new
                // RobotController(communicator);
                // SwingUtilities.invokeLater(() ->{
                // robotController = new RobotController(communicator);
                // JFrame frame = new JFrame("Robot Controller");
                // frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                // frame.add(robotController); frame.pack(); frame.setVisible(true);
                // });
                moveToCoral = false;
                // noteYawPID = new PIDController(
                // DriveConstants.YawOverrideAlignNoteKP,
                // DriveConstants.YawOverrideAlignNoteKI,
                // DriveConstants.YawOverrideAlignNoteKD);
                // noteYawPID.setSetpoint(0.0);

                // targetYawPID = new PIDController(
                // DriveConstants.TargetYawOverrideAlignNoteKP,
                // DriveConstants.TargetYawOverrideAlignNoteKI,
                // DriveConstants.TargetYawOverrideAlignNoteKD);
                // targetYawPID.setSetpoint(0.0);

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

                                        double xVelConditioned = xVelFiltered * xVelFiltered
                                                        * Math.signum(xVelFiltered);
                                        double yVelConditioned = yVelFiltered * yVelFiltered
                                                        * Math.signum(yVelFiltered);
                                        double twistConditioned = twistFiltered * twistFiltered
                                                        * Math.signum(twistFiltered);

                                        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8
                                        // m/s)
                                        double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;

                                        Logger.recordOutput("Drive/velocityMultiplier", maxVelocityMultiplier);

                                        double velocityMag = /* maxVelocityMultiplier * */ Math
                                                        .sqrt(xVelConditioned * xVelConditioned
                                                                        + yVelConditioned * yVelConditioned);
                                        if (velocityMag > 1) {
                                                xVelConditioned /= velocityMag;
                                                yVelConditioned /= velocityMag;
                                        }

                                        Logger.recordOutput("Drive/xVelConditioned", xVelConditioned);
                                        Logger.recordOutput("Drive/yVelConditioned", yVelConditioned);
                                        Logger.recordOutput("Drive/twistConditioned", twistConditioned);

                                        Logger.recordOutput("Drive/realVelX",
                                                        xVelConditioned * DriveConstants.MaxDriveableVelocity
                                                                        * maxVelocityMultiplier);
                                        Logger.recordOutput("Drive/realVelY",
                                                        yVelConditioned * DriveConstants.MaxDriveableVelocity
                                                                        * maxVelocityMultiplier);
                                        Logger.recordOutput("Drive/realVelRot",
                                                        twistConditioned * DriveConstants.MaxAngularRadiansPerSecond
                                                                        * maxVelocityMultiplier);

                                        return drive
                                                        .withVelocityX(
                                                                        xVelConditioned * DriveConstants.MaxDriveableVelocity
                                                                                        * maxVelocityMultiplier)// Drive
                                                                                                                // forward
                                                                                                                // with
                                                                                                                // negative
                                                                                                                // Y
                                                                                                                // (forward)
                                                        .withVelocityY(
                                                                        yVelConditioned * DriveConstants.MaxDriveableVelocity
                                                                                        * maxVelocityMultiplier) // Drive
                                                                                                                 // left
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // X
                                                                                                                 // (left)
                                                        .withRotationalRate(twistConditioned
                                                                        * DriveConstants.MaxAngularRadiansPerSecond
                                                                        * maxVelocityMultiplier); // Drive
                                                                                                  // counterclockwise
                                                                                                  // with negative X
                                                                                                  // (left)
                                }));

                configureAutoBuilder();
                // autoChooser = AutoBuilder.buildAutoChooser("Tests");
                // SmartDashboard.putData("Auto Mode", autoChooser);
                configurePathPlanner();

        }

        // public void stop() {
        // motor.set(0);
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
                /***************** DRIVE *****************/

                // reset the field-centric heading on left bumper press
                resetFieldRelativeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button12);
                resetFieldRelativeButton.onTrue(driveSub.runOnce(() -> driveSub.seedFieldCentric()));

                lockWheelsButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button7);
                lockWheelsButton.whileTrue(driveSub.applyRequest(() -> brake));

                // moveToCoralButton = new JoystickButton(driverController,
                // LogitechExtreme3DConstants.Button8);
                // moveToCoralButton.onTrue(new InstantCommand(() -> changeCoralVision(true)))
                // .onFalse(new InstantCommand(() -> changeCoralVision(false)));

                /***************** ARM POSITION *****************/

                moveToTravellingButton = new JoystickButton(operatorController, XboxControllerConstants.RightStick);
                moveToTravellingButton.onTrue(setArmPosTravellingCmd.command());

                moveToLoadCoralButton = new JoystickButton(operatorController, XboxControllerConstants.RightBumper);
                moveToLoadCoralButton.onTrue(setArmPosLoadCoralCmd.command());

                // moveToAlgaeButton = new JoystickButton(operatorController,
                // XboxControllerConstants.LeftTrigger);
                // moveToAlgaeButton.onTrue(setArmPosAlgaeCmd);

                removeAlgaeButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
                removeAlgaeButton.onTrue(removeAlgaeCmd.command());

                removeHighAlgaeButton = new POVButton(operatorController, XboxControllerConstants.POVLeft);
                removeHighAlgaeButton.onTrue(removeHighAlgaeCmd.command());

                moveToLevel1Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonY);
                moveToLevel1Button.onTrue(setArmPosLevel1Cmd.command());

                moveToLevel2Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
                moveToLevel2Button.onTrue(setArmPosLevel2Cmd.command());

                moveToLevel3Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonB);
                moveToLevel3Button.onTrue(setArmPosLevel3Cmd.command());

                moveToLevel4Button = new JoystickButton(operatorController, XboxControllerConstants.ButtonA);
                moveToLevel4Button.onTrue(setArmPosLevel4Cmd.command());

                startClimbButton = new JoystickButton(operatorController, XboxControllerConstants.HamburgerButton);
                startClimbButton.onTrue(startClimbingCmd.command());

                /***************** ALGAE INTAKE *****************/

                intakeAlgaeButton = new POVButton(operatorController, XboxControllerConstants.POVDown);
                intakeAlgaeButton.onTrue(intakeAlgaeCmd);

                scoreAlgaeButton = new POVButton(operatorController, XboxControllerConstants.POVRight);
                scoreAlgaeButton.onTrue(ejectAlgaeCmd).onFalse(stopAlgaeIntakeCmd);

                retractIntakeButton = new POVButton(operatorController, XboxControllerConstants.POVUp);
                retractIntakeButton.onTrue(stopAlgaeIntakeCmd);

                /***************** GRIPPER *****************/

                scoreCoralButton = new JoystickButton(operatorController, XboxControllerConstants.WindowButton);
                scoreCoralButton.onTrue(scoreCoralCmd.command());

                autoAlignAlgaeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button5);
                autoAlignAlgaeButton.whileTrue(
                                dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.AlgaeOffset, 0.04, 2)
                                                .andThen(Commands.waitUntil(() -> false)));
                // autoAlignAlgaeButton.whileTrue(
                // dynAutoBuilder.manualAlign(Constants.AutoConstants.AlgaeOffset)
                // .andThen(Commands.waitUntil(() -> false)));

                autoAlignAlgaeButton_2 = new JoystickButton(driverController, LogitechExtreme3DConstants.Button6);
                autoAlignAlgaeButton_2.whileTrue(
                                dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.AlgaeOffset, 0.04, 2)
                                                .andThen(Commands.waitUntil(() -> false)));
                // autoAlignAlgaeButton_2.whileTrue(
                // dynAutoBuilder.manualAlign(Constants.AutoConstants.AlgaeOffset)
                // .andThen(Commands.waitUntil(() -> false)));

                autoAlignLeftButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button3);
                autoAlignLeftButton.whileTrue(
                                dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.LeftOffset, 0.04, 2)
                                                .andThen(Commands.waitUntil(() -> false)));
                // autoAlignLeftButton.whileTrue(
                // dynAutoBuilder.manualAlign(Constants.AutoConstants.LeftOffset)
                // .andThen(Commands.waitUntil(() -> false)));

                autoAlignRightButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button4);
                autoAlignRightButton.whileTrue(
                                dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.RightOffset, 0.04, 2)
                                                .andThen(Commands.waitUntil(() -> false)));

                // autoAlignRightButton.whileTrue(
                // dynAutoBuilder.manualAlign(Constants.AutoConstants.RightOffset)
                // .andThen(Commands.waitUntil(() -> false)));

                // resetOdometryToVision = new JoystickButton(driverController,
                // LogitechExtreme3DConstants.Button12);
                // resetOdometryToVision.onTrue(new InstantCommand(() ->
                // driveSub.resetPose(driveSub.getPose())));

                // disableVisionButton = new JoystickButton(driverController,
                // LogitechExtreme3DConstants.Button12);
                // disableVisionButton.onTrue(new InstantCommand(() ->
                // visionSub.disableTheVision(true))).onFalse(Commands.none());

                // enableVisionButton = new JoystickButton(driverController,
                // LogitechExtreme3DConstants.Button11);
                // enableVisionButton.onTrue(new InstantCommand(() ->
                // visionSub.disableTheVision(false))).onFalse(Commands.none());

        }

        private void drivePovBindings() {
                leftPovButton = new POVButton(driverController, 270);
                leftPovButton.whileTrue(driveSub.applyRequest(() -> {
                        double povInput = driverController.getPOV();
                        double sliderInput = driverController.getThrottle();
                        Logger.recordOutput("Drive/povInput", povInput);
                        Logger.recordOutput("Drive/sliderInput", sliderInput);
                        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8
                        // m/s)
                        double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
                        double vel = DriveConstants.NudgeSpeed / maxVelocityMultiplier;
                        Logger.recordOutput("Drive/nudgeVelocity", vel);
                        ChassisSpeeds nudgeSpeeds;
                        if (armState.getCurrentState() != ArmState.Climb) {
                                nudgeSpeeds = new ChassisSpeeds(0.0, vel, 0.0);
                        } else {
                                nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0);
                        }
                        return nudge.withSpeeds(nudgeSpeeds);
                }));
                ;

                rightPovButton = new POVButton(driverController, 90);
                rightPovButton.whileTrue(driveSub.applyRequest(() -> {
                        double povInput = driverController.getPOV();
                        double sliderInput = driverController.getThrottle();
                        Logger.recordOutput("Drive/povInput", povInput);
                        Logger.recordOutput("Drive/sliderInput", sliderInput);
                        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8
                        // m/s)
                        double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
                        double vel = -1 * DriveConstants.NudgeSpeed / maxVelocityMultiplier;
                        Logger.recordOutput("Drive/nudgeVelocity", vel);
                        ChassisSpeeds nudgeSpeeds;
                        if (armState.getCurrentState() != ArmState.Climb) {
                                nudgeSpeeds = new ChassisSpeeds(0.0, vel, 0.0);
                        } else {
                                nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0);
                        }
                        return nudge.withSpeeds(nudgeSpeeds);
                }));

                upPovButton = new POVButton(driverController, 0);
                upPovButton.whileTrue(driveSub.applyRequest(() -> {
                        double povInput = driverController.getPOV();
                        double sliderInput = driverController.getThrottle();
                        Logger.recordOutput("Drive/povInput", povInput);
                        Logger.recordOutput("Drive/sliderInput", sliderInput);
                        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8
                        // m/s)
                        double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
                        double vel = DriveConstants.NudgeSpeed / maxVelocityMultiplier;
                        Logger.recordOutput("Drive/nudgeVelocity", vel);
                        ChassisSpeeds nudgeSpeeds;
                        if (armState.getCurrentState() != ArmState.Climb) {
                                nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0);
                        } else {
                                nudgeSpeeds = new ChassisSpeeds(0.0, -vel, 0.0);
                        }
                        return nudge.withSpeeds(nudgeSpeeds);
                }));

                downPovButton = new POVButton(driverController, 180);
                downPovButton.whileTrue(driveSub.applyRequest(() -> {
                        double povInput = driverController.getPOV();
                        double sliderInput = driverController.getThrottle();
                        Logger.recordOutput("Drive/povInput", povInput);
                        Logger.recordOutput("Drive/sliderInput", sliderInput);
                        // Converts from old range (1 to -1) to desired range (1 to 0.4, 4.5 m/s to 1.8
                        // m/s)
                        double maxVelocityMultiplier = (((sliderInput + 1) * (1 - 0.4)) / 2) + 0.4;
                        double vel = -1 * DriveConstants.NudgeSpeed / maxVelocityMultiplier;
                        Logger.recordOutput("Drive/nudgeVelocity", vel);
                        ChassisSpeeds nudgeSpeeds;
                        if (armState.getCurrentState() != ArmState.Climb) {
                                nudgeSpeeds = new ChassisSpeeds(vel, 0.0, 0.0);
                        } else {
                                nudgeSpeeds = new ChassisSpeeds(0.0, -vel, 0.0);
                        }
                        return nudge.withSpeeds(nudgeSpeeds);
                }));
        }

        public enum Group1State {
                Scoring,
                Removing,
                MovingElevator,
                Stopping,
                Running,
                Done,
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
                NamedCommands.registerCommand("setPositionCoralL1", setArmPosLevel1Cmd.command());
                NamedCommands.registerCommand("setPositionCoralL2", setArmPosLevel2Cmd.command());
                NamedCommands.registerCommand("setPositionCoralL3", setArmPosLevel3Cmd.command());
                NamedCommands.registerCommand("setPositionCoralL4", setArmPosLevel4Cmd.command());
                // NamedCommands.registerCommand("", setArmPosLevel4Cmd);

                NamedCommands.registerCommand("setPositionTraveling", setArmPosTravellingCmd.command());

                NamedCommands.registerCommand("setPositionLoadCoral", setArmPosLoadCoralCmd.command());
                NamedCommands.registerCommand("startOuttake", scoreCoralCmd.command());
                NamedCommands.registerCommand("startOuttakeWithoutTravelling",
                                scoreCoralCmdWithoutTravelling.command());

                // NamedCommands.registerCommand("StartAlgaeOuttake", setArmPosAlgaeCmd);

                autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio
                                                              // and
                                                              // delete them there
                SmartDashboard.putData("Selected Auto", autoChooser);
                // autoChooser.addOption("Top 3 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Top 3 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Bottom 2 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Bottom 2 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Left from Bottom 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Left from Bottom 1 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Right 1 Coral (Level3)", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Right 1 Coral (Level3)"), Set.of(driveSub)));
                // autoChooser.addOption("Top Left 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Top Left 1 Coral"), Set.of(driveSub)));

                // autoChooser.addOption("Bottom Left 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Bottom Left 1 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Top Right 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Top Right 1 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Bottom Right 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Bottom Right 1 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Top 2 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Top 2 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Bottom 3 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Bottom 3 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("Left from Top 1 Coral", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Left from Top 1 Coral"), Set.of(driveSub)));
                // autoChooser.addOption("[TEST] Box Auto", new DeferredCommand(() ->
                // driveSub.ChoreoAuto("Box Auto"), Set.of(driveSub)));
                Command commandAlgaeL4 = Commands.sequence(
                                new PositionGripper(armState, ArmState.StartAlgaePosition, elevatorSub, armSub,
                                                wristSub).command(),
                                new InstantCommand(() -> gripperSub.startAlgaeRemoval()),
                                new MoveElevator(elevatorSub,
                                                elevatorSub.getDesiredElevatorHeightInches(),
                                                Constants.ArmConstants.ElevatorAlgaeRemovalVelocity,
                                                elevatorSub.getDesiredCarriageHeightInches() + 5.0,
                                                20),
                                new InstantCommand(() -> gripperSub.stop()),
                                new PositionGripper(armState, ArmState.Running, elevatorSub, armSub, wristSub)
                                                .command(),
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Done)));

                Command commandGroup1 = Commands.sequence(
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Scoring)),
                                // wpk need to fix magic numbers
                                new ScoreCoralWithoutTravel(armState, elevatorSub, armSub, wristSub,
                                                gripperSub).command(),
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Removing)),
                                new InstantCommand(() -> gripperSub.startAlgaeRemoval()),
                                // move the elevator up to strip the algae
                                new InstantCommand(
                                                () -> Logger.recordOutput("Auto/Group1", Group1State.MovingElevator)),
                                new MoveElevator(elevatorSub,
                                                elevatorSub.getDesiredElevatorHeightInches(),
                                                Constants.ArmConstants.ElevatorAlgaeRemovalVelocity,
                                                elevatorSub.getDesiredCarriageHeightInches() + 5.0,
                                                20),
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Stopping)),
                                new InstantCommand(() -> gripperSub.stop()),
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Running)),
                                new PositionGripper(armState, ArmState.Running, elevatorSub, armSub, wristSub)
                                                .command(),
                                new InstantCommand(() -> Logger.recordOutput("Auto/Group1", Group1State.Done)));

                Command autoAlignCommandLeft = dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.LeftOffset, 3.0,
                                3.0, 0.07, 5);
                Command autoAlignCommandRight = dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.RightOffset, 3.0,
                                3.0, 0.07, 5);
                Command autoAlignCommandCenter = dynAutoBuilder.trapezoidAlign(Constants.AutoConstants.AutoOffset, 0.07,
                                5);

                autoChooser.addOption("Leave Zone",
                                new DeferredCommand(() -> driveSub.ChoreoAuto("[USED] Leave Zone"), Set.of(driveSub)));
                autoChooser.addOption("Score L1 Path",
                                new DeferredCommand(() -> driveSub.ChoreoAuto("[USED] Score L1 Path"),
                                                Set.of(driveSub)));
                autoChooser.addOption("Score L3 Path",
                                new DeferredCommand(() -> driveSub.ChoreoAuto("[USED] Score L3 Path"),
                                                Set.of(driveSub)));
                autoChooser.addOption("2-Coral-Current-Alliance",
                                new DeferredCommand(
                                                () -> driveSub.CustomChoreoAuto("[USED] 2CoralP", true,
                                                                setArmPosLevel1Cmd, commandGroup1,
                                                                scoreCoralCmd, autoAlignCommandRight,
                                                                autoAlignCommandCenter, chuteHasCoral,
                                                                new PositionGripper(armState, ArmState.Level3,
                                                                                elevatorSub, armSub, wristSub)),
                                                Set.of(driveSub)));

                autoChooser.addOption("2-Coral-Opposite-Alliance", new DeferredCommand(
                                () -> driveSub.CustomChoreoAuto("[USED] 2CoralP", false, setArmPosLevel1Cmd,
                                                commandGroup1,
                                                scoreCoralCmd, autoAlignCommandRight, autoAlignCommandCenter,
                                                chuteHasCoral,
                                                new PositionGripper(armState, ArmState.Level3, elevatorSub, armSub,
                                                                wristSub)),
                                Set.of(driveSub)));

                autoChooser.addOption("(L4) 2-Coral-Current-Alliance",
                                new DeferredCommand(
                                                () -> driveSub.CustomChoreoAutoL4("[USED] 2CoralP", true,
                                                                setArmPosLevel4Cmd, setArmPosTravellingCmd,
                                                                scoreCoralCmd, autoAlignCommandRight,
                                                                dynAutoBuilder.trapezoidAlign(
                                                                                Constants.AutoConstants.LeftOffset,
                                                                                3.0, 2.25, 0.07, 5),
                                                                chuteHasCoral,
                                                                new LoadCoralFromChute(elevatorSub, armSub, wristSub,
                                                                                gripperSub, armState)),
                                                Set.of(driveSub)));
                autoChooser.addOption("(L4) 2-Coral-Opposite-Alliance", new DeferredCommand(
                                () -> driveSub.CustomChoreoAutoL4("[USED] 2CoralP", false, setArmPosLevel4Cmd,
                                                setArmPosTravellingCmd,
                                                scoreCoralCmd, autoAlignCommandRight,
                                                dynAutoBuilder.trapezoidAlign(
                                                                Constants.AutoConstants.LeftOffset, 3.0, 2.25, 0.07, 5),
                                                chuteHasCoral,
                                                new LoadCoralFromChute(elevatorSub, armSub, wristSub, gripperSub,
                                                                armState)),
                                Set.of(driveSub)));

                autoChooser.addOption("1-Coral-L4-Right",
                                new DeferredCommand(() -> driveSub.ChoreoAuto1CoralL4("[USED] 1-Coral-L4",
                                                autoAlignCommandRight, scoreCoralCmd, setArmPosLevel4Cmd),
                                                Set.of(driveSub)));

                autoChooser.addOption("1-Coral-L4-Left",
                                new DeferredCommand(() -> driveSub.ChoreoAuto1CoralL4("[USED] 1-Coral-L4",
                                                autoAlignCommandLeft, scoreCoralCmd, setArmPosLevel4Cmd),
                                                Set.of(driveSub)));

                autoChooser.addOption("1-Coral-L4-Right-Algae",
                                new DeferredCommand(() -> driveSub.ChoreoAuto1CoralL4WithAlgae("[USED] 1-Coral-L4",
                                                autoAlignCommandRight, scoreCoralCmd, setArmPosLevel4Cmd,
                                                autoAlignCommandCenter, commandAlgaeL4), Set.of(driveSub)));

                autoChooser.addOption("1-Coral-L4-Left-Algae",
                                new DeferredCommand(
                                                () -> driveSub.ChoreoAuto1CoralL4WithAlgae("[USED] 1-Coral-L4",
                                                                autoAlignCommandLeft, scoreCoralCmd, setArmPosLevel4Cmd,
                                                                autoAlignCommandCenter, commandAlgaeL4),
                                                Set.of(driveSub)));

                // autoChooser.addOption("VisionTest", new PathPlannerAuto("TestVision"));

                Shuffleboard.getTab("Match").add("Path Name", autoChooser);

                /* LOGGING */
                // PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

                PathPlannerLogging.setLogCurrentPoseCallback(
                                (currentPose) -> {
                                        Logger.recordOutput("Auto/CurrentPose", currentPose);
                                });
                PathPlannerLogging.setLogActivePathCallback(
                                (activePath) -> {
                                        Logger.recordOutput(
                                                        "Auto/Trajectory",
                                                        activePath.toArray(new Pose2d[activePath.size()]));
                                });
                PathPlannerLogging.setLogTargetPoseCallback(
                                (targetPose) -> {
                                        Logger.recordOutput("Auto/TrajectorySetpoint", targetPose);
                                });

                // Logger.recordOutput(, nu);
        }

        public Command getAutonomousCommand() {
                if (autoChooser != null) {
                        return autoChooser.getSelected();
                }
                return null;
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
                                                                        .withWheelForceFeedforwardsX(feedforwards
                                                                                        .robotRelativeForcesXNewtons())
                                                                        .withWheelForceFeedforwardsY(feedforwards
                                                                                        .robotRelativeForcesYNewtons())),
                                        new PPHolonomicDriveController(
                                                        // TODO: These gains appears to have been massively under-tuned
                                                        // for
                                                        // reasonable PP Tracking.
                                                        // PID constants for translation
                                                        new PIDConstants(3, 0.5, 0),
                                                        // PID constants for rotation
                                                        new PIDConstants(3, 0.5, 0)),
                                        config,
                                        // Assume the path needs to be flipped for Red vs Blue, this is normally the
                                        // case
                                        () -> {
                                                // var alliance = DriverStation.getAlliance();
                                                // if (alliance.isPresent() && DriverStation.isAutonomous()) {
                                                // return alliance.get() == DriverStation.Alliance.Red;
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
}
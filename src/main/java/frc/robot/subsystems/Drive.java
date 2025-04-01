package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.commands.LoadCoralFromChute;
import frc.robot.commands.PositionGripper;
import frc.robot.commands.ScoreCoral;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Drive extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    // SysId routine for characterizing translation. This is used to find PID gains
    // for the drive motors.
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    // SysId routine for characterizing rotation. This is used to find PID gains for
    // the FieldCentricFacingAngleHeadingController. See the documentation of
    // SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /** Swerve request to apply during robot-centric path following */

    /**********************************************************************/
    /************************** CONSTRUCTORS **************************/
    /**********************************************************************/

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     * 
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Drive(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     * 
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odom loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Drive(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public void stopDrive() {

        for (int i = 0; i < 4; i++) {
            super.getModule(i).getDriveMotor().stopMotor();
            super.getModule(i).getSteerMotor().stopMotor();
        }

        // not sure if this would work better/differently:
        // for (SwerveModule<TalonFX,TalonFX,CANcoder> module : super.getModules()) {
        // module.getSteerMotor().stopMotor(); module.getDriveMotor().stopMotor(); }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        logData();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    // public void resetOdometry() {
    // this::resetPose
    // }

    public Command ChoreoAuto(String name) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromChoreoTrajectory(name);
            PathPlannerPath finalPath;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                finalPath = originalPath.flipPath();
            } else {
                finalPath = originalPath;
            }

            return AutoBuilder.followPath(finalPath)
                    .alongWith(Commands.runOnce(() -> resetPose(finalPath.getStartingHolonomicPose().get())));
        } catch (IOException e) {
            e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
        } catch (ParseException e) {
            e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
        }
        return Commands.none();
    }

    public enum AutoState {
        Waiting,
        Path1,
        Path2,
        Path3,
        Path4,
        Action1,
        Action2,
        Action3,
        Action4,
        Action5,
        Done,
    }

    public Command CustomChoreoAuto(String name, boolean mirror, PositionGripper setArmPosLevel1Cmd,
            Command sequentalCommand1, ScoreCoral scoreCoralCmd, Command autoAlignRightCommand,
            Command autoAlignCenterCommand, BooleanSupplier chuteHasCoral, PositionGripper goToL3) {
        try {
            PathPlannerPath originalPath;
            PathPlannerPath originalPath_2;
            PathPlannerPath originalPath_3;
            if (mirror) {
                originalPath = PathPlannerPath.fromChoreoTrajectory(name + "1").mirrorPath();
                originalPath_2 = PathPlannerPath.fromChoreoTrajectory(name + "2").mirrorPath();
                originalPath_3 = PathPlannerPath.fromChoreoTrajectory(name + "3").mirrorPath();
            } else {
                originalPath = PathPlannerPath.fromChoreoTrajectory(name + "1");
                originalPath_2 = PathPlannerPath.fromChoreoTrajectory(name + "2");
                originalPath_3 = PathPlannerPath.fromChoreoTrajectory(name + "3");
            }

            PathPlannerPath finalPath;
            PathPlannerPath finalPath_2;
            PathPlannerPath finalPath_3;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                finalPath = originalPath.flipPath();
                finalPath_2 = originalPath_2.flipPath();
                finalPath_3 = originalPath_3.flipPath();
            } else {
                finalPath = originalPath;
                finalPath_2 = originalPath_2;
                finalPath_3 = originalPath_3;
            }

            // return Commands.sequence(getFullCommand(finalPath), new WaitCommand(2),
            // getFullCommand(finalPath_2));
            //
            // BooleanSupplier methodRefSupplier = chuteSub::hasCoral;
            return Commands.sequence(
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path1)),
                    new ParallelCommandGroup(getFullCommandWithReset(finalPath), setArmPosLevel1Cmd.command()),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action1)),
                    autoAlignCenterCommand.withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action2)),
                    sequentalCommand1.asProxy(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path2)),
                    getFullCommand(finalPath_2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Waiting)),
                    (new WaitCommand(2)).until(chuteHasCoral)/* .andThen(new WaitCommand(0.4)) */,
                    // new SequentialCommandGroup(() -> chuteSub.hasCoral()).withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path3)),
                    getFullCommand(finalPath_3),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action3)),
                    autoAlignRightCommand.withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)),
                    goToL3.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)),

                    scoreCoralCmd.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Done))

            );

            // return Commands.sequence(new WaitCommand(1), getFullCommand(finalPath), new
            // WaitCommand(1), /*sequentalCommand1.asProxy(), sequentalCommand1.asProxy(),*/
            // new WaitCommand(3), getFullCommand(finalPath_2), new WaitCommand(1),
            // getFullCommand(finalPath_3));

        } catch (IOException e) {
            e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
        } catch (ParseException e) {
            e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
        }
        return Commands.none();
    }

    public Command ChoreoAuto1CoralL4(String name, Command autoAlignCommandRight, ScoreCoral scoreCoralCmd,
            PositionGripper setArmPosLevel4Cmd) {
        try {
            PathPlannerPath originalPath = PathPlannerPath.fromChoreoTrajectory(name);
            PathPlannerPath finalPath;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                finalPath = originalPath.flipPath();
            } else {
                finalPath = originalPath;
            }

            return Commands.sequence(
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path1)),
                    getFullCommandWithReset(finalPath),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action1)),
                    autoAlignCommandRight.withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action2)),
                    setArmPosLevel4Cmd.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action3)),
                    scoreCoralCmd.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)));
        } catch (IOException e) {
            e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
        } catch (ParseException e) {
            e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
        }
        return Commands.none();
    }


    public Command ChoreoAuto1CoralL4WithAlgae(String name, Command autoAlignCommandRight, ScoreCoral scoreCoralCmd,
    PositionGripper setArmPosLevel4Cmd, Command autoAlignCommandCenter, Command commandAlgaeL4) {
    try {
        PathPlannerPath originalPath = PathPlannerPath.fromChoreoTrajectory(name);
        PathPlannerPath finalPath;

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            finalPath = originalPath.flipPath();
        } else {
            finalPath = originalPath;
        }

        return Commands.sequence(
                new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path1)),
                getFullCommandWithReset(finalPath),
                new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action1)),
                autoAlignCommandRight.withTimeout(2),
                new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action2)),
                setArmPosLevel4Cmd.command(),
                new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action3)),
                scoreCoralCmd.command(),
                new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)),
                autoAlignCommandCenter.withTimeout(2),
                commandAlgaeL4
                

        );


    } catch (IOException e) {
        e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
    } catch (ParseException e) {
        e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
    }
    return Commands.none();
    }


    public Command CustomChoreoAutoL4(String name, boolean mirror, PositionGripper setArmPosLevel4Cmd,
            PositionGripper goToTravel, ScoreCoral scoreCoralCmd, Command autoAlignRightCommand,
            Command autoAlignLeftCommand, BooleanSupplier chuteHasCoral, LoadCoralFromChute loadCoralCommand) {
        try {
            PathPlannerPath originalPath;
            PathPlannerPath originalPath_2;
            PathPlannerPath originalPath_3;
            if (mirror) {
                originalPath = PathPlannerPath.fromChoreoTrajectory(name + "1").mirrorPath();
                originalPath_2 = PathPlannerPath.fromChoreoTrajectory(name + "2").mirrorPath();
                originalPath_3 = PathPlannerPath.fromChoreoTrajectory(name + "3").mirrorPath();
            } else {
                originalPath = PathPlannerPath.fromChoreoTrajectory(name + "1");
                originalPath_2 = PathPlannerPath.fromChoreoTrajectory(name + "2");
                originalPath_3 = PathPlannerPath.fromChoreoTrajectory(name + "3");
            }

            PathPlannerPath finalPath;
            PathPlannerPath finalPath_2;
            PathPlannerPath finalPath_3;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                finalPath = originalPath.flipPath();
                finalPath_2 = originalPath_2.flipPath();
                finalPath_3 = originalPath_3.flipPath();
            } else {
                finalPath = originalPath;
                finalPath_2 = originalPath_2;
                finalPath_3 = originalPath_3;
            }

            // return Commands.sequence(getFullCommand(finalPath), new WaitCommand(2),
            // getFullCommand(finalPath_2));
            //
            // BooleanSupplier methodRefSupplier = chuteSub::hasCoral;
            return Commands.sequence(
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path1)),
                    getFullCommandWithReset(finalPath),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action1)),
                    autoAlignRightCommand.withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action2)),
                    setArmPosLevel4Cmd.command(),
                    scoreCoralCmd.command(),
                    // goToTravel.command(),
                    // sequentalCommand1.asProxy(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path2)),
                    getFullCommand(finalPath_2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Waiting)),
                    new WaitCommand(2).until(chuteHasCoral),
                    // new SequentialCommandGroup(() -> chuteSub.hasCoral()).withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Path3)),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new WaitCommand(0.3),
                                    loadCoralCommand.withFullGoToTravel(false).command()),
                            autoAlignLeftCommand.withTimeout(4)),
                    // loadCoralCommand.command(),
                    // getFullCommand(finalPath_3),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action3)),
                    // autoAlignLeftCommand.withTimeout(2),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)),
                    setArmPosLevel4Cmd.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Action4)),
                    scoreCoralCmd.command(),
                    new InstantCommand(() -> Logger.recordOutput("Auto/State", AutoState.Done)));

            // return Commands.sequence(new WaitCommand(1), getFullCommand(finalPath), new
            // WaitCommand(1), /*sequentalCommand1.asProxy(), sequentalCommand1.asProxy(),*/
            // new WaitCommand(3), getFullCommand(finalPath_2), new WaitCommand(1),
            // getFullCommand(finalPath_3));

        } catch (IOException e) {
            e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
        } catch (ParseException e) {
            e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
        }
        return Commands.none();
    }

    private Command getFullCommandWithReset(PathPlannerPath finalPath) {
        return AutoBuilder.followPath(finalPath)
                .alongWith(Commands.runOnce(() -> resetPose(finalPath.getStartingHolonomicPose().get())));
    }

    private Command getFullCommand(PathPlannerPath finalPath) {
        return AutoBuilder.followPath(finalPath);
    }

    public Command ChoreoAutoWithoutReset(String name) {
        PathPlannerPath path /* may need to get rid of that -> */ = null;
        try {
            path = PathPlannerPath.fromChoreoTrajectory(name);
            // Do something with the path
        } catch (IOException e) {
            e.printStackTrace(); // Handle the IOException (e.g., log it or notify the user)
            path = null;
        } catch (ParseException e) {
            e.printStackTrace(); // Handle the ParseException (e.g., log it or notify the user)
            path = null;
        } catch (FileVersionException e) {
            e.printStackTrace();
        }
        return AutoBuilder.followPath(path);
    }

    private void logData() {
        Logger.recordOutput("Drive/StatesActual", getState().ModuleStates);
        Logger.recordOutput("Drive/Pose", getPose());
        // Logger.recordOutput("Drive/PoseEstimate",
        // poseEstimator.getEstimatedPosition());
        Logger.recordOutput("Drive/Heading", getState().RawHeading);
        Logger.recordOutput("Drive/Odometry/X", getPose().getX());
        Logger.recordOutput("Drive/Odometry/Y", getPose().getY());
        Logger.recordOutput("Drive/Speeds", getState().Speeds);
        Logger.recordOutput("Drive/Pitch", getPigeon2().getPitch().getValueAsDouble());
        Logger.recordOutput("Drive/Roll", getPigeon2().getRoll().getValueAsDouble());
        Logger.recordOutput("Drive/Yaw", getPigeon2().getYaw().getValueAsDouble());
        // Logger.recordOutput("Drive/CurrentSupply/FrontLeftDrive",
        // frontLeft.getDriveCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/FrontLeftTurn",
        // frontLeft.getTurnCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/FrontRightDrive",
        // frontRight.getDriveCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/FrontRightTurn",
        // frontRight.getTurnCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/BackLeftDrive",
        // backLeft.getDriveCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/BackLeftTurn",
        // backLeft.getTurnCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/BackRightDrive",
        // backRight.getDriveCurrent());
        // Logger.recordOutput("Drive/CurrentSupply/BackRightTurn",
        // backRight.getTurnCurrent());
    }

    // public Pose2d getPredictedPose() {
    // return poseEstimator.getEstimatedPosition();
    // }
}

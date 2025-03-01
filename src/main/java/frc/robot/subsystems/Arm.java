// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;

// import static edu.wpi.first.units.Units.Amp;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Robot;
import frc.robot.commands.ArmStateParameters;
import frc.robot.sim.PhysicsSim;
// import frc.robot.subsystems.Gripper.GripperState;

public class Arm extends SubsystemBase {

    private final HashMap<ArmState, ArmStateParameters> positionDictionary = new HashMap<ArmState, ArmStateParameters>();
    // include intake speeds, in dictionary, speed that intake runs at.
    // Remember to increase the capacity of the hashmap, it's default 16

    TalonFX armMotor;
    private final DCMotorSim armMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState armMotorSim;
    private final CANcoder armEncoder; // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/FusedCANcoder/src/main/java/frc/robot/Robot.java

    TalonFX carriageMotor;
    private final DCMotorSim carriageMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState carriageMotorSim;

    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    private final SparkMaxSim wristMotorSim;
    private final CANcoder wristEncoder;
    private final CANcoderSimState wristEncoderSim;
    public final ProfiledPIDController wristPIDController;
    private final DCMotorSim wristMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));
    TalonFX elevatorMotor;
    private final TalonFXSimState elevatorMotorSim;
    private final DCMotorSim elevatorMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));

    /*
    DigitalInput elevatorHallEffect;
    DIOSim elevatorHallEffectSim;
    DigitalInput carriageHallEffect;
    DIOSim carriageHallEffectSim;
    */

    // CHANGE - also need to double check that this is fine with the Algae
    // high/low/position stuff
    public enum ArmState {
        WaitingForCoral, Startup, LoadCoral, PostLoadCoral, //PreLevel1, 
        Level1, ScoreLevel1, Level2, ScoreLevel2, Level3, ScoreLevel3, Level4, ScoreLevel4, 
        StartAlgaePosition, Running, 
        // SafeToLowerArm, 
        FinishRemovingAlgae
    }

    private final Robot robot;
    // private final Drive driveSub;
    // private final Vision visionSub;
    private final Gripper gripperSub;

    private ArmState actualState = ArmState.Startup;
    public ArmState desiredState = ArmState.Startup;

    private double desiredWristAngle = 0; // CHANGE PLACEHOLDER
    private double desiredCarriageHeight = 0;
    private double desiredArmAngle = 0; // CHANGE PLACEHOLDER
    private double desiredElevatorHeight = 0;
    private double desiredGripperVelocity = 0;

    public boolean targetIsVisible = false;
    private boolean simulationInitialized = false;

    public Arm(Robot robot, Vision visionSub, Drive driveSub, Gripper gripperSub) {
        /* elevatorHallEffect = new DigitalInput(ElectronicsIDs.ElevatorHallEffect);
        carriageHallEffect = new DigitalInput(ElectronicsIDs.CarriageHallEffect); */
        armMotor = new TalonFX(ElectronicsIDs.ArmMotorID);
        armMotorSim = armMotor.getSimState();
        //armMotor.setPosition(0);
        armEncoder = new CANcoder(ElectronicsIDs.ArmEncoderID);

        carriageMotor = new TalonFX(ElectronicsIDs.CarriageMotorID);
        carriageMotorSim = carriageMotor.getSimState();
        carriageMotor.setPosition(0);

        wristMotor = new SparkMax(ElectronicsIDs.WristMotorID, MotorType.kBrushless);
        wristMotorSim = new SparkMaxSim(wristMotor, DCMotor.getNeo550((1)));
        wristEncoder = new CANcoder(ElectronicsIDs.WristEncoderID, "rio");
        wristEncoderSim = wristEncoder.getSimState();

        elevatorMotor = new TalonFX(ElectronicsIDs.ElevatorMotorID);
        elevatorMotorSim = elevatorMotor.getSimState();
        elevatorMotor.setPosition(0);

        applyAllConfigs();

        carriageMotor.setPosition(0.0) ;


        wristPIDController = new ProfiledPIDController(5.0, 0.001, 0.2, new TrapezoidProfile.Constraints(
                ArmConstants.WristMaxAngularVelocity, ArmConstants.WristMaxAngularAcceleration));
        wristPIDController.setIZone(Units.degreesToRotations(6.0));
        wristPIDController.setIntegratorRange(-0.5, 0.5) ;
        wristPIDController.setTolerance(Units.degreesToRotations(1.0));
        

        wristPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.robot = robot;
        // this.driveSub = driveSub;
        // this.visionSub = visionSub;
        this.gripperSub = gripperSub;
        initializePositionDictionary();
    }

    /** CHANGE: this version is just for testing */
    private void initializePositionDictionary() {
        // positionDictionary.put(ArmState.PreLevel1, new ArmStateParameters(0, 20, -30, 0, 0));
        positionDictionary.put(ArmState.Level1, new ArmStateParameters(0, 20, -30, 90, 0.0));
        positionDictionary.put(ArmState.ScoreLevel1, new ArmStateParameters(0, 20, -30, 90, 0.0));
        positionDictionary.put(ArmState.Level2, new ArmStateParameters(0, 12.163, 45, 0, 0));
        positionDictionary.put(ArmState.ScoreLevel2, new ArmStateParameters(0, 12.163, 10, 0, -0.1));
        positionDictionary.put(ArmState.Level3, new ArmStateParameters(7.764, 20, 60, 0, 0));
        positionDictionary.put(ArmState.ScoreLevel3, new ArmStateParameters(5.664, 20, 60, 0, -0.1));
        positionDictionary.put(ArmState.Level4, new ArmStateParameters(20, 20, 60, 0, 0));
        positionDictionary.put(ArmState.ScoreLevel4, new ArmStateParameters(25.664, 24.5, 60, 0, -0.1));
        positionDictionary.put(ArmState.WaitingForCoral, new ArmStateParameters(0, 0, -60, 90, 0));
        positionDictionary.put(ArmState.LoadCoral, new ArmStateParameters(0, 0, -75, 90, 0.5));
        positionDictionary.put(ArmState.PostLoadCoral, new ArmStateParameters(0, 0, -75, 90, 0));
        positionDictionary.put(ArmState.Startup, new ArmStateParameters(0, 0, 90, 0, 0));
        positionDictionary.put(ArmState.Running, new ArmStateParameters(0, 0, 90, 0, 0));
        positionDictionary.put(ArmState.StartAlgaePosition, new ArmStateParameters(0, 0, -30, 0, -0.2));
        positionDictionary.put(ArmState.FinishRemovingAlgae, new ArmStateParameters(0, 0, 60, 0, -0.5));
        // positionDictionary.put(ArmState.SafeToLowerArm, new ArmStateParameters(0, 0, 90, 0, 0));
    }

    private void realInitializePositionDictionary() {
        // CHANGE all these magic #s (except for wrist)
        // need to change speeds to all of these (right now assuming grabing is + and
        // release is -)
        // The PreL1 and L1 values below are approximations. The PreL1 arm angle and
        // carriage ht are
        // to get the arm beyond the elevator frame before rotating the coral in the
        // the gripper (which happens in the L1 state).
        // Our target ht for the gripper over the trough is 25".
        // VALUES IN DEGREES & INCHES. Convert as necessary.
        // NOTE: All carriage heights should be reduced by 4" once the carriage resting,
        // aka zero,
        // level is reset upward to adjust for the chute.
        // TEST: For L2-4 scoring states, e.g. ArmState.Level3, we are not changing the
        // arm angle
        // during the scoring motion to begin testing. Eventually we will lower the
        // angle to
        // facilitate scoring. Not changing angle is to insure we don't damage the arm
        // by running it into the reef. We simply lower the carriage a small amount (2")
        // to
        // see if the end of the coral is extended too far, too little or just enough to
        // slide
        // over the top of the reef branch.

        // Starting from Running, move the arm angle so that the coral in the gripper is
        // outside the
        // body of the elevator before rotating the wrist. This is to avoid the coral
        // colliding
        // with the elevator frame when turned horizontal.
        // wpk positionDictionary.put(ArmState.PreLevel1, new ArmStateParameters(0, 22.25, 45, 0, 0));
        // We should never enter this state without having gone through PreLevel1 first.
        // Once the coral is safely rotated, proceed to the position for ejecting the
        // coral
        // into the trough.
        // The carriage height and arm angle here are chosen so that the coral in the
        // gripper would
        // be elevated above and slightly over coral resting in the front row of the
        // trough.
        // The gripper eject speed needs to be high enough that the ejected coral will
        // roll off of
        // any coral already in the front row of the trough and to the second row.
        // STRETCH: We could use odometry to let us know when clear enough from the reef
        // to move
        // the arm to WaitingForCoral automatically. Same for L2-4. It risks hitting
        // another
        // obstacle like algae or another bot, but is relatively safe as we are likely
        // backing
        // away from the reef leaving empty space in front of us. We need ~4" of
        // clearance to
        // be safe (relative to the reef).
        positionDictionary.put(ArmState.Level1, new ArmStateParameters(0, 20, -30, 90, -0.2));
        positionDictionary.put(ArmState.ScoreLevel1, new ArmStateParameters(0, 20, -30, 90, 0.0));
        // PreL2 is initiated by the operator's L2 button and positions the coral above
        // (1")
        // the L2 reef branch ready for scoring/release.
        // PreL3/4 are similar.
        positionDictionary.put(ArmState.Level2, new ArmStateParameters(0, 12.163, 60, 0, 0));
        // L2 is the scoring state. It causes the movement of the carriage & arm down
        // over the L2
        // reef branch. It also softly (velocity to be tested), ejects the coral from
        // the
        // gripper.
        // TBD: It may also rotate the arm more lower to help the coral get on the
        // reef branch assuming geometry allows.
        // This state should NEVER be entered unless PreL2 is complete.
        // Level3/4 states are similar.
        positionDictionary.put(ArmState.ScoreLevel2, new ArmStateParameters(0, 10, 60, 0, -0.1));
        positionDictionary.put(ArmState.Level3, new ArmStateParameters(1.264, 26.5, 60, 0, 0));
        positionDictionary.put(ArmState.ScoreLevel3, new ArmStateParameters(1.264, 24.5, 60, 0, -0.1));
        positionDictionary.put(ArmState.Level4, new ArmStateParameters(25.664, 26.5, 60, 0, 0));
        positionDictionary.put(ArmState.ScoreLevel4, new ArmStateParameters(25.664, 24.5, 60, 0, -0.1));
        // WFC positions the arm so that the gripper is hovering just above where coral
        // will arrive in
        // the chute.
        // After any coral scoring action, we will return to this state.
        // TBD: Auto or by operator action?
        // TEST: How safely can this occur when we are still parked at the reef? Since
        // the arm
        // rotates down past horizontal, it could collide with the reef.
        positionDictionary.put(ArmState.WaitingForCoral, new ArmStateParameters(0, 18.29, -60, 0, 0));
        // LoadCoral lowers the gripper onto coral in the chute and spins the gripper
        // wheels to
        // pull in the coral.
        // This state should never be entered unless previously in PreLoadCoral.
        // This state should always be followed by PostLoadCoral.
        // TBD: Is the above true if the load fails, i.e. the gripper fails to pick up
        // the coral?
        positionDictionary.put(ArmState.LoadCoral, new ArmStateParameters(0, 15.69, -75, 0, 0.5));
        // PostLoadCoral raises the carriage enough so the the gripper with loaded coral
        // clears the
        // outside edge of the chute. This allows the arm to rotate up with coral
        // without that
        // coral colliding with the chute. It allows us to return to the Running
        // (travelling)
        // position safely.
        // This state should NEVER be entered unless previously in LoadCoral.
        // TBD: Do we automatically go to Running from here? If so, can we be sure that
        // when
        // the arm rotates up, that we won't run the gripper into another bot?
        // Or is operator input required to go to running?
        positionDictionary.put(ArmState.PostLoadCoral, new ArmStateParameters(0, 18, -75, 0, 0));
        // positionDictionary.put(ArmState.AlgaeLow, new ArmStateParameters(0, 0, 0, 0,
        // -1));
        // positionDictionary.put(ArmState.AlgaeHigh, new ArmStateParameters(0, 0, 0, 0,
        // -1));
        positionDictionary.put(ArmState.Startup, new ArmStateParameters(0, 0, 0, 0, 0));
        // Running, aka "travelling", has the carriage down and the arm up with gripper
        // rotated for L2-4
        // We expect to move here after PostLoadCoral, so whenever there is coral being
        // carried.
        positionDictionary.put(ArmState.Running, new ArmStateParameters(0, 0, 90, 0, 0));
        // FRA is the state/position we move to as we perform algae removal. Here, we
        // move from low
        // to high.
        // TBD: How high is high enough? For starters, we max out elev+carr. Optimize to
        // lower level.
        // TBD: Do we need to keep the gripper wheels ejecting the whole time we are
        // moving to this
        // position? If so, leave them running in this position. They will stop on next
        // transition.
        positionDictionary.put(ArmState.StartAlgaePosition, new ArmStateParameters(0, 22.25, -30, 90, -0.2));
        positionDictionary.put(ArmState.FinishRemovingAlgae, new ArmStateParameters(25, 26.5, 60, 90, -0.5));
        // positionDictionary.put(ArmState.SafeToLowerArm, new ArmStateParameters(0, 0, 0, 90, 0));

    }

    private void setDesiredAnglesAndHeights() {
        var val = positionDictionary.get(desiredState);
        desiredWristAngle = val.getWristAngle();
        desiredCarriageHeight = val.getCarriageHeight();
        desiredArmAngle = val.getArmAngle();
        desiredElevatorHeight = val.getElevatorHeight();
        desiredGripperVelocity = val.getGripperVelocity();
        setArmAngle(desiredArmAngle);
        setWristAngle(desiredWristAngle);
        setCarriageHeightInches(desiredCarriageHeight);
        setElevatorHeightInches(desiredElevatorHeight);
    }

    public boolean hasCompletedMovement() {
        return desiredState == actualState;
    }

    @Override
    public void periodic() {

        if (robot.isEnabled()) {
            setDesiredAnglesAndHeights();
        }

        if (isAtDesiredState()) {
            actualState = desiredState;
        }
        // Once we've moved to a precursor state, then force the change to the
        // state for the follow-on motion.
        // The following cases are situations where two or more movements must occur
        // in sequence. Once we've moved to a precursor state, then force the change to
        // the state for the follow-on motion.
        // NOTE: This logic should probably occur at the Command level.

        // if (desiredState == ArmState.PreLevel1 && isWithinArmAngleTolerance()) {
        //     setDesiredState(ArmState.Level1);
        // }

        if (desiredState == ArmState.LoadCoral && gripperSub.hasCoral()) {
            setDesiredState(ArmState.PostLoadCoral);
        }
        if (desiredState == ArmState.PostLoadCoral && isWithinCarriageHeightTolerance()) {
            setDesiredState(ArmState.Running);
        }

        // boundsCheck();

        boundsCheck();

        logData();

        BaseStatusSignal.refreshAll(
                armMotor.getFault_FusedSensorOutOfSync(false),
                armMotor.getStickyFault_FusedSensorOutOfSync(false),
                armMotor.getFault_RemoteSensorDataInvalid(false),
                armMotor.getStickyFault_RemoteSensorDataInvalid(false),
                armMotor.getPosition(false),
                armMotor.getVelocity(false),
                armEncoder.getPosition(false),
                armEncoder.getVelocity(false),
                armMotor.getRotorPosition(false));

        // Shuffleboard.getTab("Match").add("Can See Tag", targetIsVisible);
        // Shuffleboard.getTab("Match").add("Desired Shooter Angle", desiredWristAngle);
    }

    public void setWristAngle(double desiredDegrees) {
        final double currentRotations = wristEncoder.getAbsolutePosition().getValueAsDouble();   
        final double wristOutput = wristPIDController.calculate(currentRotations, Units.degreesToRotations(desiredDegrees));
        // Logger.recordOutput("Arm/WristAngle/PIDOutput", wristOutput);
        wristMotor.setVoltage(wristOutput);
    }

    public void setArmAngle(double desiredDegrees) {
        MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        request.EnableFOC = Constants.IsFocEnabled;
        armMotor.setControl(request);
        // this.desiredArmAngle = desiredDegrees; // Why is this commented out
    }

    public double getWristEncoderDegrees() {
        return wristEncoder.getAbsolutePosition().getValue().in(Degrees);
    }

    public double getArmEncoderDegrees() {
        return armEncoder.getAbsolutePosition().getValue().in(Degrees);
    }

    public double getArmTalonEncoderDegrees() {
        return armMotor.getPosition().getValue().in(Degrees);
    }

    public void setElevatorHeightInches(double desiredInches) {
        double rotations = ((desiredInches /*- ArmConstants.StartingElevatorHeight*/))
                * ArmConstants.RotationsPerElevatorInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        request.EnableFOC = Constants.IsFocEnabled;
        elevatorMotor.setControl(request.withSlot(0));
    }

    public void setCarriageHeightInches(double desiredInches) {
        double rotations = ((desiredInches /*- ArmConstants.StartingCarriageHeight*/))
                * ArmConstants.RotationsPerCarriageInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        request.EnableFOC = Constants.IsFocEnabled;
        carriageMotor.setControl(request.withSlot(0));
    }

    public double getElevatorHeightInches() {
        double elevatorHeight = ((elevatorMotor.getPosition().getValue().in(Rotations)
                / ArmConstants.RotationsPerElevatorInch))
        /* + ArmConstants.StartingElevatorHeight */;
        return elevatorHeight;
    }

    public double getCarriageHeightInches() {
        double carriageHeight = ((carriageMotor.getPosition().getValue().in(Rotations)
                / ArmConstants.RotationsPerCarriageInch));
        /* + ArmConstants.StartingCarriageHeight; */
        return carriageHeight;
    }

    /* ARM STATES */

    public ArmState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(ArmState newState) {
        desiredState = newState;
    }

    public ArmState getArmState() {
        return actualState;
    }

    private boolean isAtDesiredState() {
        if (isWithinWristAngleTolerance() && isWithinArmAngleTolerance() && isWithinElevatorHeightTolerance()
                && isWithinCarriageHeightTolerance()) {
            return true;
        } else {
            return false;
        }
    }

    public double getDesiredGripperVel() {
        return desiredGripperVelocity;
    }

    /* TOLERANCES */

    private boolean isWithinWristAngleTolerance() {
        boolean withinTolerance = (getWristEncoderDegrees() >= desiredWristAngle - ArmConstants.WristAngleTolerance)
                && (getWristEncoderDegrees() <= desiredWristAngle + ArmConstants.WristAngleTolerance);
        return withinTolerance;
    }

    private boolean isWithinArmAngleTolerance() {
        boolean withinTolerance = (getArmEncoderDegrees() >= desiredArmAngle - ArmConstants.ArmAngleTolerance)
                && (getArmEncoderDegrees() <= desiredArmAngle + ArmConstants.ArmAngleTolerance);
        return withinTolerance;
    }

    private boolean isWithinElevatorHeightTolerance() {
        boolean withinTolerance = (getElevatorHeightInches() >= desiredElevatorHeight
                - ArmConstants.ElevatorHeightTolerance) &&
                (getElevatorHeightInches() <= desiredElevatorHeight + ArmConstants.ElevatorHeightTolerance);
        return withinTolerance;
    }

    private boolean isWithinCarriageHeightTolerance() {
        boolean withinTolerance = (getCarriageHeightInches() >= desiredCarriageHeight
                - ArmConstants.CarriageHeightTolerance) &&
                (getCarriageHeightInches() <= desiredCarriageHeight + ArmConstants.CarriageHeightTolerance);
        return withinTolerance;
    }

    public boolean isAvailableToGoToReef() {
        ArmState desiredState = getDesiredState();
        // return (desiredState == ArmState.Running || desiredState == ArmState.Level1
        // || desiredState == ArmState.Level2
        // || desiredState == ArmState.Level3 || desiredState == ArmState.Level4
        // || desiredState == ArmState.PreLevel1 || desiredState == ArmState.PreLevel2
        // || desiredState == ArmState.PreLevel3 || desiredState == ArmState.PreLevel4
        // || desiredState == ArmState.AlgaeHigh || desiredState == ArmState.AlgaeLow);
        return (!(desiredState == ArmState.WaitingForCoral) && !(desiredState == ArmState.LoadCoral));
    }

    public boolean isAvailableToGoToCoralStation() {
        ArmState desiredState = getDesiredState();
        return (desiredState == ArmState.WaitingForCoral || desiredState == ArmState.LoadCoral);
    }

    /* SAFETY */

    public void stopElevatorMotor() {
        elevatorMotor.set(0);
    }

    public void stopCarriageMotor() {
        carriageMotor.stopMotor();
        carriageMotor.set(0);
    }

    public void stopArmMotor() {
        armMotor.set(0);
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    /*
    public boolean carriageIsAtBottom() {
        return !carriageHallEffect.get();
    }

    public boolean elevatorIsAtBottom() {
        return !elevatorHallEffect.get();
    }
        */

    /** Makes sure we never go past our limits of motion */
    private void boundsCheck() {
        if ((getElevatorHeightInches() <= 0 && elevatorMotor.getVelocity().getValueAsDouble() < 0 && !isWithinElevatorHeightTolerance()) ||
                (getElevatorHeightInches() > ArmConstants.MaxElevatorHeight
                        && elevatorMotor.getVelocity().getValueAsDouble() > 0
                        && !isWithinElevatorHeightTolerance())) {
            stopElevatorMotor();
        }

        if ((getCarriageHeightInches() <= 0
                && carriageMotor.getVelocity().getValueAsDouble() < 0
                && !isWithinCarriageHeightTolerance()) ||
                (getCarriageHeightInches() >= ArmConstants.MaxCarriageHeight
                        && carriageMotor.getVelocity().getValueAsDouble() > 0
                        && !isWithinCarriageHeightTolerance())) {
            stopCarriageMotor();
        }

        if ((getArmEncoderDegrees() <= ArmConstants.MinArmAngle
                && armMotor.getVelocity().getValueAsDouble() < 0
                && !isWithinArmAngleTolerance()) ||
                (getArmEncoderDegrees() >= ArmConstants.MaxArmAngle
                && armMotor.getVelocity().getValueAsDouble() > 0
                && !isWithinArmAngleTolerance())) { 
            stopArmMotor();
        }

        if ((getWristEncoderDegrees() <= ArmConstants.MinWristAngle
                && wristMotor.get() < 0
                && !isWithinWristAngleTolerance()) ||
                (getWristEncoderDegrees() >= ArmConstants.MaxWristAngle
                        && wristMotor.get() > 0
                        && !isWithinWristAngleTolerance())) {
            stopWristMotor();
        }

        // // Not sure if we need this
        // if ((getArmEncoderDegrees() <= ArmConstants.MinArmAngle // need to fix these
        // constant values
        // && armMotor.getVelocity().getValueAsDouble() < 0) ||
        // (getArmEncoderDegrees() >= ArmConstants.MaxArmAngle // need to fix these
        // constant values
        // && armMotor.getVelocity().getValueAsDouble() > 0)) {
        // stopArmMotor();
        // }

        // // Not sure if we need this
        // if ((getWristEncoderDegrees() <= ArmConstants.MinWristAngle // need to fix
        // these constant values
        // && wristMotor.getEncoder().getVelocity() < 0) ||
        // (getWristEncoderDegrees() >= ArmConstants.MaxWristAngle // need to fix these
        // constant values
        // && wristMotor.getEncoder().getVelocity() > 0)) {
        // stopWristMotor();
        // }
    }

    private void logData() {
        Logger.recordOutput("Arm/StateActual", actualState);
        Logger.recordOutput("Arm/StateDesired", desiredState);

        Logger.recordOutput("Arm/AtDesiredState", isAtDesiredState());
        Logger.recordOutput("Arm/HasCompletedMovment", hasCompletedMovement());
        Logger.recordOutput("Arm/Tolerances/WithinWristTolerance", isWithinWristAngleTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinArmTolerance", isWithinArmAngleTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinElevatorTolerance", isWithinElevatorHeightTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinCarriageTolerance", isWithinCarriageHeightTolerance());

        // SparkMax Config
        Logger.recordOutput("Arm/WristAngle/DegreesDesired", desiredWristAngle);
        Logger.recordOutput("Arm/WristAngle/DegreesCANcoder", getWristEncoderDegrees());
        Logger.recordOutput("Arm/WristAngle/RotationsCANcoder", wristEncoder.getAbsolutePosition().getValue());
        Logger.recordOutput("Arm/WristAngle/PositionError", wristPIDController.getPositionError());
        Logger.recordOutput("Arm/WristAngle/VelocityError", wristPIDController.getVelocityError());
        Logger.recordOutput("Arm/WristAngle/AccumulatedError", wristPIDController.getAccumulatedError());
        Logger.recordOutput("Arm/WristAngle/SetPointPosition", wristPIDController.getSetpoint().position);
        Logger.recordOutput("Arm/WristAngle/SetPointPosition", wristPIDController.getGoal().position);

        
        // Logger.recordOutput("Arm/WristAngle/VoltageActual",
        // wristMotor.getEncoder().getVelocity());
        // Logger.recordOutput("Arm/WristAngle/RPSActual",
        // wristMotor.getEncoder().getVelocity());
        if (Robot.isSimulation()) {
            Logger.recordOutput("Arm/WristAngle/SimulatedPosition", wristMotorSim.getPosition());
        }

        Logger.recordOutput("Arm/ArmAngle/DegreesDesired", desiredArmAngle);
        Logger.recordOutput("Arm/ArmAngle/DegreesCANcoder", getArmEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/RotationsCANcoder", armEncoder.getAbsolutePosition().getValue());
        Logger.recordOutput("Arm/ArmAngle/DegreesTalon", getArmTalonEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/VoltageActual", armMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ArmAngle/ClosedLoopError", armMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ArmAngle/ProportionalOutput", armMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Arm/ArmAngle/DerivativeOutput", armMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Arm/ArmAngle/IntegratedOutput", armMotor.getClosedLoopIntegratedOutput().getValue());

        Logger.recordOutput("Arm/ArmAngle/ClosedLoopOutput", armMotor.getClosedLoopOutput().getValue());
        Logger.recordOutput("Arm/ArmAngle/ClosedLoopFF", armMotor.getClosedLoopFeedForward().getValue());

        Logger.recordOutput("Arm/ArmAngle/ClosedLoopReference", Units.rotationsToDegrees(armMotor.getClosedLoopReference().getValue()));

        Logger.recordOutput("Arm/ArmAngle/MotionMagicIsRunning", armMotor.getMotionMagicIsRunning().getValue());

        Logger.recordOutput("Arm/ArmAngle/SupplyCurrent", armMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ArmAngle/StatorCurrent", armMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Arm/ArmAngle/RPSActual", armMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ArmAngle/AccelerationActual", armMotor.getAcceleration().getValue());

        Logger.recordOutput("Arm/ElevatorHeight/InchesDesired", desiredElevatorHeight);
        Logger.recordOutput("Arm/ElevatorHeight/InchesActual", getElevatorHeightInches());
        Logger.recordOutput("Arm/ElevatorHeight/VoltageActual", elevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ClosedLoopError",
                elevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ProportionalOutput",
                elevatorMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/DerivativeOutput",
                elevatorMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/IntegratedOutput",
                elevatorMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/SupplyCurrent", elevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/TempC", elevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ControlMode", elevatorMotor.getControlMode().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/RotationsActual",
                elevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/ElevatorHeight/RotationsDesired",
                elevatorMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/RPSActual", elevatorMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/AccelerationActual",
                elevatorMotor.getAcceleration().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/StatorCurrent", elevatorMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ClosedLoopFF", elevatorMotor.getClosedLoopFeedForward().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ClosedLoopReference", elevatorMotor.getClosedLoopReference().getValue()/ ArmConstants.RotationsPerElevatorInch);
        Logger.recordOutput("Arm/ElevatorHeight/MotionMagicIsRunning", elevatorMotor.getMotionMagicIsRunning().getValue());


        Logger.recordOutput("Arm/Carriage/InchesDesired", desiredCarriageHeight);
        Logger.recordOutput("Arm/Carriage/InchesActual", getCarriageHeightInches());
        Logger.recordOutput("Arm/Carriage/VoltageActual", carriageMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/Carriage/ClosedLoopError", carriageMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/Carriage/ProportionalOutput",
                carriageMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Arm/Carriage/DerivativeOutput",
                carriageMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Arm/Carriage/IntegratedOutput",
                carriageMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Arm/Carriage/SupplyCurrent", carriageMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/Carriage/TempC", carriageMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Arm/Carriage/ControlMode", carriageMotor.getControlMode().getValue());
        Logger.recordOutput("Arm/Carriage/RotationsActual", carriageMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/Carriage/RotationsDesired", carriageMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Arm/Carriage/RPSActual", carriageMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/Carriage/AccelerationActual", carriageMotor.getAcceleration().getValue());
        Logger.recordOutput("Arm/Carriage/StatorCurrent", carriageMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Arm/Carriage/ClosedLoopOutput", carriageMotor.getClosedLoopOutput().getValue());
        Logger.recordOutput("Arm/Carriage/ClosedLoopFF", carriageMotor.getClosedLoopFeedForward().getValue());
        Logger.recordOutput("Arm/Carriage/ClosedLoopReference", carriageMotor.getClosedLoopReference().getValue()/ ArmConstants.RotationsPerCarriageInch);
        Logger.recordOutput("Arm/Carriage/MotionMagicIsRunning", carriageMotor.getMotionMagicIsRunning().getValue());

    }

    /* CONFIG */

    public void applyAllConfigs() {
        applyArmEncoderConfigs();
        applyArmMotorConfigs(InvertedValue.CounterClockwise_Positive);
        applyWristEncoderConfigs();
        applyElevatorMotorConfigs(elevatorMotor, "elevatorMotor", InvertedValue.Clockwise_Positive);
        applyCarriageMotorConfigs(carriageMotor, "carriageMotor", InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake, NeutralModeValue.Brake);

        /* CHANGE (this was commented out durring testing) */
        // wristMotorConfig.closedLoop
        // .pidf(ArmConstants.WristKP, ArmConstants.WristKI, ArmConstants.WristKD,
        // ArmConstants.WristFF)
        // .iZone(ArmConstants.WristIZone)
        // .outputRange(-1, 1);
        wristMotorConfig.inverted(true);
        wristMotor.configure(wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void applyArmMotorConfigs(InvertedValue inversion) {
        
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.ArmAngleKP2;
        talonConfigs.Slot0.kI = ArmConstants.ArmAngleKI2;
        talonConfigs.Slot0.kD = ArmConstants.ArmAngleKD2;
        talonConfigs.Slot0.kV = ArmConstants.ArmAngleKV2;
        talonConfigs.Slot0.kG = ArmConstants.ArmAngleKG2;
        talonConfigs.Slot0.kS = ArmConstants.ArmAngleKS2;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ArmAngleCruiseSpeed;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.ArmAngleAcceleration;
        motionMagicConfigs.MotionMagicJerk = ArmConstants.ArmAngleJerk;

        talonConfigs.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.SensorToMechanismRatio = 1.0;
        talonConfigs.Feedback.RotorToSensorRatio = ArmConstants.ArmAngleGearRatio;


        applyMotorConfigs(armMotor, "armMotor", talonConfigs, inversion);
    }

    private void applyElevatorMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.ElevatorKP.get();
        talonConfigs.Slot0.kI = ArmConstants.ElevatorKI.get();
        talonConfigs.Slot0.kD = ArmConstants.ElevatorKD.get();
        talonConfigs.Slot0.kV = ArmConstants.ElevatorKV.get();
        talonConfigs.Slot0.kG = ArmConstants.ElevatorKG.get();
        talonConfigs.Slot0.kS = ArmConstants.ElevatorKS.get();
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        /*
         * talonConfigs.Slot1.kP = ShooterMountConstants.ClimbKP;
         * talonConfigs.Slot1.kI = ShooterMountConstants.ClimbKI;
         * talonConfigs.Slot1.kD = ShooterMountConstants.ClimbKD;
         * talonConfigs.Slot1.kV = ShooterMountConstants.ClimbFF;
         * talonConfigs.Slot1.kG = ShooterMountConstants.ClimbKG;\
         */
        talonConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;

        double rotationsPerSecond = ArmConstants.ElevatorCruiseVelocity
                * ArmConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        double rotationsPerSecondPerSecond = (ArmConstants.ElevatorAcceleration
                * ArmConstants.RotationsPerElevatorInch) / 0.25;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecondPerSecond;

        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecondPerSecond / 0.1;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }




    private void applyCarriageMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.CarriageKPTP.get();
        talonConfigs.Slot0.kI = ArmConstants.CarriageKITP.get();
        talonConfigs.Slot0.kD = ArmConstants.CarriageKDTP.get();
        talonConfigs.Slot0.kV = ArmConstants.CarriageKVTP.get();
        talonConfigs.Slot0.kG = ArmConstants.CarriageKGTP.get();
        talonConfigs.Slot0.kS = ArmConstants.CarriageKSTP.get();
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        /*
         * talonConfigs.Slot1.kP = ShooterMountConstants.ClimbKP;
         * talonConfigs.Slot1.kI = ShooterMountConstants.ClimbKI;
         * talonConfigs.Slot1.kD = ShooterMountConstants.ClimbKD;
         * talonConfigs.Slot1.kV = ShooterMountConstants.ClimbFF;
         * talonConfigs.Slot1.kG = ShooterMountConstants.ClimbKG;\
         */
        // talonConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;

        double rotationsPerSecond = ArmConstants.CarriageCruiseVelocity
                * ArmConstants.RotationsPerCarriageInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        double rotationsPerSecondPerSecond = (ArmConstants.CarriageAcceleration
                * ArmConstants.RotationsPerCarriageInch) / 0.25;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecondPerSecond;

        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecondPerSecond / 0.1;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }



    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        /* SET & APPLY INVERSION CONFIGS */

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT CONFIGS */

        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ArmConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motor + " error code: " + status.toString());
        }
    }

    private void applyArmEncoderConfigs() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;

        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ArmConstants.ArmAngleCANcoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor = magnetConfig;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = armEncoder.getConfigurator().apply(CANcoderConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply CANcoder configs to arm angle encoder, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = armEncoder.getConfigurator().apply(magnetConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply magnet configs to arm angle encoder, error code: " + status.toString());
        }
    }

    private void applyWristEncoderConfigs() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        var CANcoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;

        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ArmConstants.WristAngleCANcoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfiguration.MagnetSensor = magnetConfig;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = wristEncoder.getConfigurator().apply(CANcoderConfiguration, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply CANcoder configs to wrist angle encoder, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = wristEncoder.getConfigurator().apply(magnetConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply magnet configs to wrist angle encoder, error code: " + status.toString());
        }
    }

    private void setNeutralMode(NeutralModeValue armMotorMode, NeutralModeValue elevatorMotorMode,
            NeutralModeValue carriageMotorMode) {
        armMotor.setNeutralMode(armMotorMode);
        elevatorMotor.setNeutralMode(elevatorMotorMode);
        carriageMotor.setNeutralMode(carriageMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(carriageMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(elevatorMotor, 0.001);

        double wristEncoderAngle = Units.degreesToRotations(desiredWristAngle);
        wristEncoderSim.setRawPosition(wristEncoderAngle);

        /* 
        elevatorHallEffectSim = new DIOSim(elevatorHallEffect);
        carriageHallEffectSim = new DIOSim(carriageHallEffect); */
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        // Following pattern from:
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html
        // armMotorSim = armMotor.getSimState();
        // var carriageMotorSim = carriageMotor.getSimState();

        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var armVoltage = armMotorSim.getMotorVoltageMeasure();

        armMotorModel.setInputVoltage(armVoltage.in(Volts));
        armMotorModel.update(0.02);

        armMotorSim.setRawRotorPosition(armMotorModel.getAngularPositionRotations());
        armMotorSim.setRotorVelocity(armMotorModel.getAngularVelocity());

        // The armEncoder needs to be synchronized from the motor simulation model
        // This is because the talonConfigs.Feedback.FeedbackRemoteSensorID is set to
        // use the
        // the encoder.
        var armEncoderSim = armEncoder.getSimState();
        armEncoderSim.setVelocity(armMotorModel.getAngularVelocityRadPerSec());
        armEncoderSim.setRawPosition(armMotorModel.getAngularPositionRotations());

        // Carriage Motor Sim

        carriageMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double carriageVoltage = carriageMotorSim.getMotorVoltage();
        carriageMotorModel.setInputVoltage(carriageVoltage);
        carriageMotorModel.update(0.02);
        carriageMotorSim.setRotorVelocity(carriageMotorModel.getAngularVelocityRPM() / 60.0);
        carriageMotorSim.setRawRotorPosition(carriageMotorModel.getAngularPositionRotations());

        // Elevator Motor Sim

        elevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = elevatorMotorSim.getMotorVoltage();
        elevatorMotorModel.setInputVoltage(leftVoltage);
        elevatorMotorModel.update(0.02);
        elevatorMotorSim.setRotorVelocity(elevatorMotorModel.getAngularVelocityRPM() / 60.0);
        elevatorMotorSim.setRawRotorPosition(elevatorMotorModel.getAngularPositionRotations());

        // Wrist Motor Sim

        wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        wristMotorModel.setInputVoltage(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        wristMotorModel.update(0.02);
        wristMotorSim.iterate(wristMotorModel.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
        wristEncoderSim.setVelocity(wristMotorModel.getAngularVelocityRadPerSec());
        wristEncoderSim.setRawPosition(wristMotorModel.getAngularPositionRotations());
    
        /* 
        if (desiredElevatorHeight == ArmConstants.MaxElevatorHeight && isWithinElevatorHeightTolerance()) {
            elevatorHallEffectSim.setValue(false);
        } else {
            elevatorHallEffectSim.setValue(true);
        }

        if (desiredCarriageHeight == ArmConstants.MaxCarriageHeight && isWithinCarriageHeightTolerance()) {
            carriageHallEffectSim.setValue(false);
        } else {
            carriageHallEffectSim.setValue(true);
        }
        */
    }
}

// TODO: NEED TO SETUP RUNNING-BETWEEN POSITION
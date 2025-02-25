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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import frc.robot.subsystems.Gripper.GripperState;

public class Arm extends SubsystemBase {

    private final HashMap<ArmState, ArmStateParameters> positionDictionary = new HashMap<ArmState, ArmStateParameters>();
    // include intake speeds, in dictionary, speed that intake runs at.
    // Remember to increase the capacity of the hashmap, it's default 16

    TalonFX armMotor;
    private final DCMotorSim armMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState armMotorSim;
    private final CANcoder armEncoder;

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
    private final DCMotorSim wristMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));
    TalonFX elevatorMotor;
    private final TalonFXSimState elevatorMotorSim;
    private final DCMotorSim elevatorMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));

    // CHANGE - also need to double check that this is fine with the Algae high/low/position stuff
    public enum ArmState {
        WaitingForCoral, Startup, LoadCoral, PostLoadCoral, PreLevel1, Level1, Level2, ScoreLevel2, Level3, ScoreLevel3, Level4, ScoreLevel4, AlgaePosition, Running, SafeToLowerArm, FinishRemovingAlgae
    }

    private final Drive driveSub;
    private final Vision visionSub;
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

    public Arm(Vision visionSub, Drive driveSub, Gripper gripperSub) {
        // bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        armMotor = new TalonFX(ElectronicsIDs.ArmMotorID);
        armMotorSim = armMotor.getSimState();
        armMotor.setPosition(0);
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

        applyArmMotorConfigs(InvertedValue.CounterClockwise_Positive);
        applyArmEncoderConfigs();
        applyWristEncoderConfigs();
        applyElevatorMotorConfigs(elevatorMotor, "elevatorMotor", InvertedValue.CounterClockwise_Positive);
        applyElevatorMotorConfigs(carriageMotor, "carriageMotor", InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);

        wristMotorConfig.closedLoop
                .pidf(ArmConstants.WristKP, ArmConstants.WristKI, ArmConstants.WristKD, ArmConstants.WristFF)
                .iZone(ArmConstants.WristIZone)
                .outputRange(-1, 1);
        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.driveSub = driveSub;
        this.visionSub = visionSub;
        this.gripperSub = gripperSub;
        initializePositionDictionary();
    }

    private void initializePositionDictionary() {
        // CHANGE all these magic #s (except for wrist)
        // need to change speeds to all of these (right now assuming grabing is + and
        // release is -)
        // The PreL1 and L1 values below are approximations. The PreL1 arm angle and
        // carriage ht are
        // to get the arm beyond the elevator frame before rotating the coral in the
        // the gripper (which happens in the L1 state).
        // Our target ht for the gripper over the trough is 25".
        // VALUES IN DEGREES & INCHES. Convert as necessary.
        positionDictionary.put(ArmState.PreLevel1, new ArmStateParameters(0, 22.25, 45, 90, -1));
        positionDictionary.put(ArmState.Level1, new ArmStateParameters(0, 22.25, -30, 0, -1));
        positionDictionary.put(ArmState.Level2, new ArmStateParameters(0, 12.163, 60, 90, 0));
        positionDictionary.put(ArmState.ScoreLevel2, new ArmStateParameters(0, 10, 45, 90, -0.2));
        positionDictionary.put(ArmState.Level3, new ArmStateParameters(1.264, 26.5, 60, 90, 0));
        positionDictionary.put(ArmState.ScoreLevel3, new ArmStateParameters(1.264, 24.5, 60, 90, -0.2));
        positionDictionary.put(ArmState.Level4, new ArmStateParameters(25.664, 26.5, 60, 90, 0));
        positionDictionary.put(ArmState.ScoreLevel4, new ArmStateParameters(25.664, 24.5, 60, 90, -0.2));
        positionDictionary.put(ArmState.WaitingForCoral, new ArmStateParameters(0, 18.29, -60, 90, 1));
        positionDictionary.put(ArmState.LoadCoral, new ArmStateParameters(0, 15.69, -75, 90, 1));
        positionDictionary.put(ArmState.PostLoadCoral, new ArmStateParameters(0, 18, -75, 90, 1));
        positionDictionary.put(ArmState.AlgaePosition, new ArmStateParameters(0, 0, 0, 0, -1));
        positionDictionary.put(ArmState.Startup, new ArmStateParameters(0, 0, 0, 90, 0));
        positionDictionary.put(ArmState.Running, new ArmStateParameters(0, 0, 90, 90, -1));
        positionDictionary.put(ArmState.FinishRemovingAlgae, new ArmStateParameters(25, 26.5, 0, 0, -1));
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
        setElevatorHeightInches(desiredElevatorHeight);
        setCarriageHeightInches(desiredCarriageHeight);
    }

    public boolean hasCompletedMovement() {
        return desiredState == actualState;
    }

    @Override
    public void periodic() {

        setDesiredAnglesAndHeights();
        if (isAtDesiredState()) {
            actualState = desiredState;
        }
        // Once we've moved to a precursor state, then force the change to the
        // state for the follow-on motion.
        if (desiredState == ArmState.PreLevel1 && isWithinArmAngleTolerance()) {
            setDesiredState(ArmState.Level1);
        }
        if (desiredState == ArmState.LoadCoral && (gripperSub.getState() == GripperState.carryingCoral)) {
            setDesiredState(ArmState.PostLoadCoral);
        }
        if (desiredState == ArmState.PostLoadCoral && isWithinCarriageHeightTolerance()) {
            setDesiredState(ArmState.Running);
        }

        boundsCheck();

        boundsCheck();

        logData();

        // Shuffleboard.getTab("Match").add("Can See Tag", targetIsVisible);
        // Shuffleboard.getTab("Match").add("Desired Shooter Angle", desiredWristAngle);
    }

    public void setWristAngle(double desiredDegrees) {
        wristMotor.getClosedLoopController().setReference(Units.degreesToRotations(desiredDegrees),
                ControlType.kPosition);
    }

    public void setArmAngle(double desiredDegrees) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        armMotor.setControl(request);
        // this.desiredArmAngle = desiredDegrees;
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
        elevatorMotor.setControl(request.withSlot(0));
    }

    public void setCarriageHeightInches(double desiredInches) {
        double rotations = ((desiredInches /*- ArmConstants.StartingCarriageHeight*/))
                * ArmConstants.RotationsPerCarriageInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
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
        // return (desiredState == ArmState.Running || desiredState == ArmState.Level1 || desiredState == ArmState.Level2
        //         || desiredState == ArmState.Level3 || desiredState == ArmState.Level4
        //         || desiredState == ArmState.PreLevel1 || desiredState == ArmState.PreLevel2
        //         || desiredState == ArmState.PreLevel3 || desiredState == ArmState.PreLevel4
        //         || desiredState == ArmState.AlgaeHigh || desiredState == ArmState.AlgaeLow);
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
        carriageMotor.set(0);
    }

    public void stopArmMotor() {
        armMotor.set(0);
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    /** Makes sure we never go past our limits of motion */
    private void boundsCheck() {
        if ((getElevatorHeightInches() <= 0 && elevatorMotor.getVelocity().getValueAsDouble() < 0) ||
            (getElevatorHeightInches() > ArmConstants.MaxElevatorHeight && elevatorMotor.getVelocity().getValueAsDouble() > 0)) {
            stopElevatorMotor();
        }

        if ((getCarriageHeightInches() <= 0
                && carriageMotor.getVelocity().getValueAsDouble() < 0) ||
                (getCarriageHeightInches() == ArmConstants.MaxCarriageHeight
                        && carriageMotor.getVelocity().getValueAsDouble() > 0)) {
            stopCarriageMotor();
        }

        // // Not sure if we need this
        // if ((getArmEncoderDegrees() <= ArmConstants.MinArmAngle // need to fix these constant values
        //         && armMotor.getVelocity().getValueAsDouble() < 0) ||
        //         (getArmEncoderDegrees() >= ArmConstants.MaxArmAngle // need to fix these constant values
        //                 && armMotor.getVelocity().getValueAsDouble() > 0)) {
        //     stopArmMotor();
        // }

        // // Not sure if we need this
        // if ((getWristEncoderDegrees() <= ArmConstants.MinWristAngle // need to fix these constant values
        //         && wristMotor.getEncoder().getVelocity() < 0) ||
        //         (getWristEncoderDegrees() >= ArmConstants.MaxWristAngle // need to fix these constant values
        //                 && wristMotor.getEncoder().getVelocity() > 0)) {
        //     stopWristMotor();
        // }
    }

    private void logData() {
        Logger.recordOutput("Arm/StateActual", actualState);
        Logger.recordOutput("Arm/StateDesired", desiredState);

        Logger.recordOutput("Arm/AtDesiredState", isAtDesiredState());
        Logger.recordOutput("Arm/Tolerances/WithinWristTolerance", isWithinWristAngleTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinArmTolerance", isWithinArmAngleTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinElevatorTolerance", isWithinElevatorHeightTolerance());
        Logger.recordOutput("Arm/Tolerances/WithinCarriageTolerance", isWithinCarriageHeightTolerance());

        // SparkMax Config
        Logger.recordOutput("Arm/WristAngle/DegreesDesired", desiredWristAngle);
        Logger.recordOutput("Arm/WristAngle/DegreesCANcoder", getWristEncoderDegrees());
        Logger.recordOutput("Arm/WristAngle/RotationsCANcoder", wristEncoder.getAbsolutePosition().getValue());
        // Logger.recordOutput("Arm/WristAngle/VoltageActual",
        // wristMotor.getEncoder().getVelocity());
        // Logger.recordOutput("Arm/WristAngle/RPSActual",
        // wristMotor.getEncoder().getVelocity());
        Logger.recordOutput("Arm/WristAngle/SimulatedPosition", wristMotorSim.getPosition());

        Logger.recordOutput("Arm/ArmAngle/DegreesDesired", desiredArmAngle);
        Logger.recordOutput("Arm/ArmAngle/DegreesCANcoder", getArmEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/RotationsCANcoder", armEncoder.getAbsolutePosition().getValue());
        Logger.recordOutput("Arm/ArmAngle/DegreesTalon", getArmTalonEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/VoltageActual", armMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ArmAngle/ClosedLoopError", armMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ArmAngle/SupplyCurrent", armMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ArmAngle/RPSActual", armMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ArmAngle/AccelerationActual", armMotor.getAcceleration().getValue());

        Logger.recordOutput("Arm/ElevatorHeight/InchesDesired", desiredElevatorHeight);
        Logger.recordOutput("Arm/ElevatorHeight/InchesActual", getElevatorHeightInches());
        Logger.recordOutput("Arm/ElevatorHeight/VoltageActual", elevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/ClosedLoopError",
                elevatorMotor.getClosedLoopError().getValue());
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

        Logger.recordOutput("Arm/CarriageHeight/InchesDesired", desiredCarriageHeight);
        Logger.recordOutput("Arm/CarriageHeight/InchesActual", getCarriageHeightInches());
        Logger.recordOutput("Arm/CarriageHeight/VoltageActual", carriageMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/CarriageHeight/ClosedLoopError", carriageMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/CarriageHeight/SupplyCurrent", carriageMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/CarriageHeight/TempC", carriageMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Arm/CarriageHeight/ControlMode", carriageMotor.getControlMode().getValue());
        Logger.recordOutput("Arm/CarriageHeight/RotationsActual", carriageMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/CarriageHeight/RotationsDesired", carriageMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Arm/CarriageHeight/RPSActual", carriageMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/CarriageHeight/AccelerationActual", carriageMotor.getAcceleration().getValue());

    }

    /* CONFIG */

    private void applyArmMotorConfigs(InvertedValue inversion) {
        
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.ArmAngleKP;
        talonConfigs.Slot0.kI = ArmConstants.ArmAngleKI;
        talonConfigs.Slot0.kD = ArmConstants.ArmAngleKD;
        talonConfigs.Slot0.kV = ArmConstants.ArmAngleFF;
        talonConfigs.Slot0.kG = ArmConstants.ArmAngleKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ArmAngleCruiseSpeed;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.ArmAngleAcceleration;
        motionMagicConfigs.MotionMagicJerk = ArmConstants.ArmAngleJerk;

        talonConfigs.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        applyMotorConfigs(armMotor, "armMotor", talonConfigs, inversion);
    }

    private void applyElevatorMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.ElevatorKP;
        talonConfigs.Slot0.kI = ArmConstants.ElevatorKI;
        talonConfigs.Slot0.kD = ArmConstants.ElevatorKD;
        talonConfigs.Slot0.kV = ArmConstants.ElevatorFF;
        talonConfigs.Slot0.kG = ArmConstants.ElevatorKG;
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

        double rotationsPerSecond = ArmConstants.ElevatorCruiseInchesPerSec
                * ArmConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        double rotationsPerSecondPerSecond = (ArmConstants.ElevatorInchesPerSecPerSec
                * ArmConstants.RotationsPerElevatorInch) / 0.25;
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
        var CANcoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;

        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ArmConstants.ArmAngleCANcoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfiguration.MagnetSensor = magnetConfig;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = armEncoder.getConfigurator().apply(CANcoderConfiguration, 0.05);
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

    private void setNeutralMode(NeutralModeValue armMotorMode, NeutralModeValue elevatorMotorMode) {
        armMotor.setNeutralMode(armMotorMode);
        elevatorMotor.setNeutralMode(elevatorMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(carriageMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(elevatorMotor, 0.001);

        double wristEncoderAngle = Units.degreesToRotations(desiredWristAngle);
        wristEncoderSim.setRawPosition(wristEncoderAngle);
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
    }
}

// TODO: NEED TO SETUP RUNNING-BETWEEN POSITION
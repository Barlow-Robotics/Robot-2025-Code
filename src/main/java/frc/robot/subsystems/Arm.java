 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.HashMap;

// import static edu.wpi.first.units.Units.Amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;
import frc.robot.commands.ElevatorState;
import frc.robot.sim.PhysicsSim;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final HashMap<ArmState, ElevatorState> positionDictionary = new HashMap<ArmState, ElevatorState>();
    // include intake speeds, in dictionary, speed that intake runs at. 
    // Remember to increase the capacity of the hashmap, it's default 16

    TalonFX armMotor;
    private final DCMotorSim armMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
        DCMotor.getKrakenX60Foc(1));
    private final CANcoder armEncoder;

    TalonFX carriageMotor;
    private final DCMotorSim carriageMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    private final SparkMaxSim wristMotorSim;
    private final CANcoder wristEncoder; 
    private final CANcoderSimState wristEncoderSim;
    private final DCMotorSim wristMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));
    
    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    public enum ArmState {
        WaitingForCoral, Startup, CoralAuto, LoadCoral, Level1, Level2, Level3, Level4, AlgaeHigh, AlgaeLow, Running
    }

    private final Drive driveSub;
    private final Vision visionSub;

    private ArmState actualState = ArmState.Startup;
    public ArmState desiredState = ArmState.Startup;

    private double desiredWristAngle = 0; // CHANGE PLACEHOLDER
    private double desiredCarriageHeight = ArmConstants.StartingCarriageHeight;
    private double desiredArmAngle = 0; // CHANGE PLACEHOLDER
    private double desiredElevatorHeight = ArmConstants.StartingElevatorHeight;

    public boolean targetIsVisible = false;
    private boolean simulationInitialized = false;

    public Arm(Vision visionSub, Drive driveSub) {
        // bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        armMotor = new TalonFX(ElectronicsIDs.ArmMotorID);
        //armMotorSim = armMotor.getSimState();
        armMotor.setPosition(0);
        
        carriageMotor = new TalonFX(ElectronicsIDs.CarriageMotorID);
        // carriageMotorSim = armMotor.getSimState();
        carriageMotor.setPosition(0);

        wristMotor = new SparkMax(ElectronicsIDs.WristMotorID, MotorType.kBrushless);
        wristMotorSim = new SparkMaxSim(wristMotor, DCMotor.getNeo550((1)));
        wristEncoder = new CANcoder(ElectronicsIDs.WristEncoderID, "rio");
        wristEncoderSim = wristEncoder.getSimState();

        armEncoder = new CANcoder(ElectronicsIDs.ArmEncoderID);
        
        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();
        leftElevatorMotor.setPosition(0);

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState(); 

        applyArmMotorConfigs(InvertedValue.CounterClockwise_Positive);
        applyWristEncoderConfigs(); 
        applyElevatorMotorConfigs(leftElevatorMotor, "leftElevatorMotor", InvertedValue.CounterClockwise_Positive);
        applyElevatorMotorConfigs(rightElevatorMotor, "rightElevatorMotor", InvertedValue.Clockwise_Positive);
        applyElevatorMotorConfigs(carriageMotor, "carriageMotor", InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        wristMotorConfig.closedLoop
            .pidf(ArmConstants.WristKP, ArmConstants.WristKI, ArmConstants.WristKD, ArmConstants.WristFF)
            .iZone(ArmConstants.WristIZone)
            .outputRange(-1, 1);        
        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.driveSub = driveSub;
        this.visionSub = visionSub;
        initializePositionDictionary();
    }
    private void initializePositionDictionary() {
        // CHANGE all these magic #s (except for wrist)
        // all elevator heights should be above ~24 (ElevatorStartingHeight)
        positionDictionary.put(ArmState.Level1, new ElevatorState(50, 0, 0, 0));
        positionDictionary.put(ArmState.Level2, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.Level3, new ElevatorState(30, 0, 0, 90));
        positionDictionary.put(ArmState.Level4, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.WaitingForCoral, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.LoadCoral, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.AlgaeLow, new ElevatorState(0, 0, 0, 0));
        positionDictionary.put(ArmState.AlgaeHigh, new ElevatorState(0, 0, 0, 0));
        positionDictionary.put(ArmState.Startup, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.CoralAuto, new ElevatorState(0, 0, 0, 90));
        positionDictionary.put(ArmState.Running, new ElevatorState(0, 0, 0, 90));
    }

    private void setDesiredAnglesAndHeights() {
        var val = positionDictionary.get(desiredState);
        desiredWristAngle = val.getWristAngle();
        desiredCarriageHeight = val.getCarriageHeight();
        desiredArmAngle = val.getArmAngle();
        desiredElevatorHeight = val.getElevatorHeight();
        setWristAngle(desiredWristAngle);
        setElevatorHeightInches(leftElevatorMotor, desiredElevatorHeight);
        setCarriageHeightInches(carriageMotor,desiredCarriageHeight);
        setArmAngle(desiredArmAngle);
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


        logData();

        // Shuffleboard.getTab("Match").add("Can See Tag", targetIsVisible);
        // Shuffleboard.getTab("Match").add("Desired Shooter Angle", desiredWristAngle);
    }

    public void setWristAngle(double desiredDegrees) {
        wristMotor.getClosedLoopController().setReference(Units.degreesToRadians(desiredDegrees), ControlType.kPosition);
    }

    public void stopWristMotor() {
        wristMotor.stopMotor();
    }
    
    public void setArmAngle(double desiredDegrees) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        armMotor.setControl(request);
        this.desiredArmAngle = desiredDegrees;
    }

    public void stopArmMotor() {
        armMotor.set(0);
    }

    public double getWristEncoderDegrees() {
        return Units.rotationsToDegrees(wristEncoder.getAbsolutePosition().getValue().baseUnitMagnitude());
    }

    public double getArmEncoderDegrees() {
        return Units.rotationsToDegrees(armEncoder.getAbsolutePosition().getValue().baseUnitMagnitude());
    }

    public double getArmTalonEncoderDegrees() {
        return Units.rotationsToDegrees(armMotor.getPosition().getValue().baseUnitMagnitude());
    }

    public void setElevatorHeightInches(TalonFX motor, double desiredInches) {
        double rotations = ((desiredInches - ArmConstants.StartingElevatorHeight) / 2)
        * ArmConstants.RotationsPerElevatorInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        motor.setControl(request.withSlot(0));
    }

    public void setCarriageHeightInches(TalonFX motor, double desiredInches) {
        double rotations = ((desiredInches - ArmConstants.StartingCarriageHeight) / 2)
        * ArmConstants.RotationsPerCarriageInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        motor.setControl(request.withSlot(0));
    }

    public double getElevatorHeightInches() {
        double elevatorHeight = ((leftElevatorMotor.getPosition().getValue().baseUnitMagnitude()
                / ArmConstants.RotationsPerElevatorInch) * 2)
                + ArmConstants.StartingElevatorHeight;
        return elevatorHeight;
    }
    public double getCarriageHeightInches() {
        double carriageHeight = ((carriageMotor.getPosition().getValue().baseUnitMagnitude()
                / ArmConstants.RotationsPerElevatorInch) * 2)
                + ArmConstants.StartingCarriageHeight;
        return carriageHeight;
    }

    public void setBasePosition(double height) {
        armMotor.setPosition(height);
    }

    /* ARM STATES */

    public ArmState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(ArmState newState) {
        if (newState == ArmState.Running) {
            return;
        }
        if (newState != desiredState) {
            actualState = ArmState.Running;
        }
        desiredState = newState;
    }

    public ArmState getArmState() {
        return actualState;
    }

    private boolean isAtDesiredState() {
        if (isWithinWristAngleTolerance() && isWithinArmAngleTolerance() && isWithinElevatorHeightTolerance() && isWithinCarriageHeightTolerance()) {
            return true;
        } else {
            return false;
        }
    }

    /* TOLERANCES */

    private boolean isWithinWristAngleTolerance() {
        boolean withinTolerence = (getWristEncoderDegrees() >= desiredWristAngle - ArmConstants.WristAngleTolerance)
                && (getWristEncoderDegrees() <= desiredWristAngle + ArmConstants.WristAngleTolerance);
        return withinTolerence;
    }

    private boolean isWithinArmAngleTolerance() {
        boolean withinTolerence = (getArmEncoderDegrees() >= desiredArmAngle - ArmConstants.ArmAngleTolerance)
                && (getArmEncoderDegrees() <= desiredArmAngle + ArmConstants.ArmAngleTolerance);
        return withinTolerence;
    }

    private boolean isWithinElevatorHeightTolerance() {
        boolean withinTolerence = (getElevatorHeightInches() >= desiredElevatorHeight - ArmConstants.ElevatorHeightTolerance) &&
                (getElevatorHeightInches() <= desiredElevatorHeight + ArmConstants.ElevatorHeightTolerance);
        return withinTolerence;
    }

    private boolean isWithinCarriageHeightTolerance() {
        boolean withinTolerence = (getCarriageHeightInches() >= desiredCarriageHeight - ArmConstants.CarriageHeightTolerance) &&
                (getCarriageHeightInches() <= desiredCarriageHeight + ArmConstants.CarriageHeightTolerance);
        return withinTolerence;
    }

    private void logData() {
        Logger.recordOutput("Arm/StateActual", actualState);
        Logger.recordOutput("Arm/StateDesired", desiredState);

        // SparkMax Config
        Logger.recordOutput("Arm/WristAngle/DegreesDesired", desiredWristAngle);
        Logger.recordOutput("Arm/WristAngle/DegreesCANCoder", getWristEncoderDegrees());
        Logger.recordOutput("Arm/WristAngle/RotationsCANCoder", wristEncoder.getAbsolutePosition().getValue());
        // Logger.recordOutput("Arm/WristAngle/VoltageActual", wristMotor.getEncoder().getVelocity());
        // Logger.recordOutput("Arm/WristAngle/RPSActual", wristMotor.getEncoder().getVelocity());
        Logger.recordOutput("Arm/WristAngle/SimulatedPosition", wristMotorSim.getPosition());

        Logger.recordOutput("Arm/ArmAngle/DegreesDesired", desiredArmAngle);
        Logger.recordOutput("Arm/ArmAngle/DegreesCANCoder", getArmEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/RotationsCANCoder", armEncoder.getAbsolutePosition().getValue());
        Logger.recordOutput("Arm/ArmAngle/DegreesTalon", getArmTalonEncoderDegrees());
        Logger.recordOutput("Arm/ArmAngle/VoltageActual", armMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ArmAngle/ClosedLoopError", armMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ArmAngle/SupplyCurrent", armMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ArmAngle/RPSActual", armMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ArmAngle/AccelerationActual", armMotor.getAcceleration().getValue());

        Logger.recordOutput("Arm/ElevatorHeight/InchesDesired", desiredElevatorHeight);
        Logger.recordOutput("Arm/ElevatorHeight/InchesActual", getElevatorHeightInches());
        Logger.recordOutput("Arm/ElevatorHeight/Left/VoltageActual", leftElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/ClosedLoopError", leftElevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/SupplyCurrent", leftElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/TempC", leftElevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/ControlMode", leftElevatorMotor.getControlMode().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/RotationsActual", leftElevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/ElevatorHeight/Left/RotationsDesired", leftElevatorMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/RPSActual", leftElevatorMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Left/AccelerationActual", leftElevatorMotor.getAcceleration().getValue());
        
        Logger.recordOutput("Arm/ElevatorHeight/Right/VoltageActual", rightElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/ClosedLoopError", rightElevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/SupplyCurrent", rightElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/TempC", rightElevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/RotationsActual", rightElevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Arm/ElevatorHeight/Right/RotationsDesired", rightElevatorMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/RPSActual", rightElevatorMotor.getVelocity().getValue());
        Logger.recordOutput("Arm/ElevatorHeight/Right/AccelerationActual", rightElevatorMotor.getAcceleration().getValue());
        
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
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ArmAngleCruiseRotationsPerSec;
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
        talonConfigs.Slot1.kP = ShooterMountConstants.ClimbKP;
        talonConfigs.Slot1.kI = ShooterMountConstants.ClimbKI;
        talonConfigs.Slot1.kD = ShooterMountConstants.ClimbKD;
        talonConfigs.Slot1.kV = ShooterMountConstants.ClimbFF;
        talonConfigs.Slot1.kG = ShooterMountConstants.ClimbKG;\
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

    private void applyWristEncoderConfigs() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        var canCoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorDiscontinuityPoint = 0.5;
        
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ArmConstants.AngleCANCoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfiguration.MagnetSensor = magnetConfig;
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = wristEncoder.getConfigurator().apply(canCoderConfiguration, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply CANCoder configs to angle encoder, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = wristEncoder.getConfigurator().apply(magnetConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply magnet configs to angle encoder, error code: " + status.toString());
        }
    }

    private void setNeutralMode(NeutralModeValue armMotorMode, NeutralModeValue elevatorMotorMode) {
        armMotor.setNeutralMode(armMotorMode); 
        leftElevatorMotor.setNeutralMode(elevatorMotorMode);
        rightElevatorMotor.setNeutralMode(elevatorMotorMode);                                                                                                 
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(carriageMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001);

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
        var armMotorSim = armMotor.getSimState();
        var carriageMotorSim = carriageMotor.getSimState();

        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        var armVoltage = armMotorSim.getMotorVoltageMeasure();

        armMotorModel.setInputVoltage(armVoltage.in(Volts));
        armMotorModel.update(0.02);

        armMotorSim.setRawRotorPosition(armMotorModel.getAngularPositionRotations());
        armMotorSim.setRotorVelocity(armMotorModel.getAngularVelocity());

        // The armEncoder needs to be synchronized from the motor simulation model
        // This is because the talonConfigs.Feedback.FeedbackRemoteSensorID is set to use the
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

        leftElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftElevatorMotorSim.getMotorVoltage();
        leftElevatorMotorModel.setInputVoltage(leftVoltage);
        leftElevatorMotorModel.update(0.02);
        leftElevatorMotorSim.setRotorVelocity(leftElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        leftElevatorMotorSim.setRawRotorPosition(leftElevatorMotorModel.getAngularPositionRotations());

        rightElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightElevatorMotorSim.getMotorVoltage();
        rightElevatorMotorModel.setInputVoltage(rightVoltage);
        rightElevatorMotorModel.update(0.02);
        rightElevatorMotorSim.setRotorVelocity(rightElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        
        // Wrist Motor Sim
        
        wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        wristMotorModel.setInputVoltage(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        wristMotorModel.update(0.02);
        wristMotorSim.iterate(wristMotorModel.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
        wristEncoderSim.setVelocity(wristMotorModel.getAngularVelocityRadPerSec());
        wristEncoderSim.setRawPosition(wristMotorModel.getAngularPositionRotations());
    }
}
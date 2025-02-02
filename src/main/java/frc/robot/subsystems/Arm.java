 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj.simulation.DIOSim;
//import edu.wpi.first.wpilibj.simulation.ElevatorSim;
//import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;
//import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    TalonFX armMotor;
    private final TalonFXSimState armMotorSim;
    
    private final DCMotorSim armMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    TalonFX carriageMotor;
    private final TalonFXSimState carriageMotorSim;
    
    private final DCMotorSim carriageMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    SparkMax wristMotor;
    private final SparkMaxSim wristMotorSim;
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    public final SparkClosedLoopController wristPidController;
    private final DCMotorSim wristMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1), DCMotor.getNeo550(1));

    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));
    
    private final CANcoder wristEncoder; 
    private final CANcoderSimState wristEncoderSim;

    private final CANcoder armEncoder;
    private final CANcoderSimState armEncoderSim;

    public enum ArmState {
        MovingToPos, LoadCoral, Level1, Level2, Level3, Level4, AlgaeHigh, AlgaeLow
    }

    private final Drive driveSub;
    private final Vision visionSub;

    private ArmState actualState = ArmState.MovingToPos;
    public ArmState desiredState = ArmState.MovingToPos;

    private double desiredWristAngle = 0; // CHANGE PLACEHOLDER
    private double desiredCarriageHeight = 0;
    private double desiredArmAngle = 0; // CHANGE PLACEHOLDER
    private double desiredElevatorHeight = 0;

    public boolean targetIsVisible = false;
    private boolean simulationInitialized = false;

    public Arm(Vision visionSub, Drive driveSub) {
        // bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        armMotor = new TalonFX(ElectronicsIDs.ArmMotorID);
        armMotorSim = armMotor.getSimState();
        armMotor.setPosition(0);
        
        carriageMotor = new TalonFX(ElectronicsIDs.CarriageMotorID);
        carriageMotorSim = armMotor.getSimState();
        carriageMotor.setPosition(0);

        wristMotor = new SparkMax(ElectronicsIDs.WristMotorID, MotorType.kBrushless);
        wristMotorSim = new SparkMaxSim(wristMotor, DCMotor.getNeo550((1)));
        wristEncoder = new CANcoder(ElectronicsIDs.WristEncoderID, "rio");
        wristEncoderSim = wristEncoder.getSimState();

        armEncoder = new CANcoder(ElectronicsIDs.ArmEncoderID);
        armEncoderSim = armEncoder.getSimState();
        
        wristPidController = wristMotor.getClosedLoopController();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();
        leftElevatorMotor.setPosition(0);

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState(); 

        applyArmMotorConfigs(InvertedValue.Clockwise_Positive);
        applyAngleEncoderConfigs(); 
        applyElevatorMotorConfigs(leftElevatorMotor, "leftElevatorMotor", InvertedValue.CounterClockwise_Positive);
        applyElevatorMotorConfigs(rightElevatorMotor, "rightElevatorMotor", InvertedValue.Clockwise_Positive);
        applyElevatorMotorConfigs(carriageMotor, "carriageMotor", InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);
        this.driveSub = driveSub;
        this.visionSub = visionSub;
        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        wristMotorConfig.closedLoop
            .pidf(ArmConstants.WristKP, ArmConstants.WristKI, ArmConstants.WristKD, ArmConstants.WristFF)
            .iZone(ArmConstants.WristIZone)
            .outputRange(-1, 1);        
    }

    private void setDesiredAnglesAndHeights() {

        switch (desiredState) {
            case MovingToPos: 
                break;
            case LoadCoral: 
                desiredWristAngle = ArmConstants.CoralWristAngle;
                desiredElevatorHeight = ArmConstants.CoralElevatorHeight;
                desiredCarriageHeight = ArmConstants.CoralCarriageHeight;
                desiredArmAngle = ArmConstants.CoralArmAngle;
                break;
            case Level1:
                desiredWristAngle = ArmConstants.Level1WristAngle;
                desiredElevatorHeight = ArmConstants.Level1ElevatorHeight;
                desiredCarriageHeight = ArmConstants.Level1CarriageHeight;
                desiredArmAngle = ArmConstants.Level1ArmAngle;
                break;
            case Level2:
                desiredWristAngle = ArmConstants.Level2WristAngle;
                desiredElevatorHeight = ArmConstants.Level2ElevatorHeight;
                desiredCarriageHeight = ArmConstants.Level2CarriageHeight;
                desiredArmAngle = ArmConstants.Level2ArmAngle;
                break;
            case Level3:
                desiredWristAngle = ArmConstants.Level3WristAngle;
                desiredElevatorHeight = ArmConstants.Level3ElevatorHeight;
                desiredCarriageHeight = ArmConstants.Level3CarriageHeight;
                desiredArmAngle = ArmConstants.Level3ArmAngle;
                break;
            case Level4:
                desiredWristAngle = ArmConstants.Level4WristAngle;
                desiredElevatorHeight = ArmConstants.Level4ElevatorHeight;
                desiredCarriageHeight = ArmConstants.Level4CarriageHeight;
                desiredArmAngle = ArmConstants.Level4ArmAngle;
                break;
            case AlgaeHigh:
                desiredWristAngle = ArmConstants.AlgaeHighWristAngle;
                desiredElevatorHeight = ArmConstants.AlgaeHighElevatorHeight;
                desiredCarriageHeight = ArmConstants.AlgaeHighCarriageHeight;
                desiredArmAngle = ArmConstants.AlgaeHighArmAngle;        
                break;
            case AlgaeLow:
               desiredWristAngle = ArmConstants.AlgaeLowWristAngle;
               desiredElevatorHeight = ArmConstants.AlgaeLowElevatorHeight;
               desiredCarriageHeight = ArmConstants.AlgaeLowCarriageHeight;
               desiredArmAngle = ArmConstants.AlgaeLowArmAngle;  
                break;
        }
        setWristAngle(desiredWristAngle);
        setElevatorHeightInches(leftElevatorMotor, desiredElevatorHeight);
        setCarriageHeightInches(carriageMotor, desiredCarriageHeight);
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
        wristPidController.setReference(desiredDegrees, ControlType.kPosition);
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
        if (newState == ArmState.MovingToPos) {
            return;
        }
        if (newState != desiredState) {
            actualState = ArmState.MovingToPos;
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
        Logger.recordOutput("Arm/WristAngle/VoltageActual", wristMotor.getEncoder().getVelocity());
        Logger.recordOutput("Arm/WristAngle/RPSActual", wristMotor.getEncoder().getVelocity());

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
        talonConfigs.Slot0.kP = ArmConstants.AngleKP;
        talonConfigs.Slot0.kI = ArmConstants.AngleKI;
        talonConfigs.Slot0.kD = ArmConstants.AngleKD;
        talonConfigs.Slot0.kV = ArmConstants.AngleFF;
        talonConfigs.Slot0.kG = ArmConstants.AngleKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.AngleCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.AngleAcceleration;
        motionMagicConfigs.MotionMagicJerk = ArmConstants.AngleJerk;

        talonConfigs.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
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

    private void applyAngleEncoderConfigs() {
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

        double encoderAngle = Units.degreesToRotations(this.desiredWristAngle);
        wristEncoderSim.setRawPosition(encoderAngle);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        armMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double armVoltage = armMotorSim.getMotorVoltage();
        armMotorModel.setInputVoltage(armVoltage);
        armMotorModel.update(0.02);
        armMotorSim.setRotorVelocity(armMotorModel.getAngularVelocityRPM() / 60.0);
        armMotorSim.setRawRotorPosition(armMotorModel.getAngularPositionRotations());

        carriageMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double carriageVoltage = armMotorSim.getMotorVoltage();
        carriageMotorModel.setInputVoltage(carriageVoltage);
        carriageMotorModel.update(0.02);
        carriageMotorSim.setRotorVelocity(carriageMotorModel.getAngularVelocityRPM() / 60.0);
        carriageMotorSim.setRawRotorPosition(carriageMotorModel.getAngularPositionRotations());

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
        
        wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        double wristVoltage = wristMotorSim.getBusVoltage();
        wristMotorModel.setInputVoltage(wristVoltage);
        wristMotorModel.update(0.02);
        wristMotorSim.setVelocity(wristMotorModel.getAngularVelocityRPM() / 60.0);

        double currentWristAngle = getWristEncoderDegrees();
        double delta = desiredWristAngle - currentWristAngle;
        delta = Math.min(Math.abs(delta), 5.0) * Math.signum(delta);
        wristEncoder.setPosition(Units.degreesToRotations(currentWristAngle + delta));

    }
}
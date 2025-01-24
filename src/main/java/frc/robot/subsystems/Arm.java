// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
// import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    TalonFX armMotor;
    private final TalonFXSimState armMotorSim;
    
    private final DCMotorSim armMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1), DCMotor.getKrakenX60Foc(1));

    SparkMax wristMotor;
    private final SimDeviceSim wristMotorSim;
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    public final SparkClosedLoopController wristPidController;
        
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
    
    private final CANcoder wristEncoder; // needs an encoder
    private final CANcoderSimState wristEncoderSim; // CHANGE needed? never used

    public enum ArmState {
        Home, LoadCoral, Level1, Level2, Level3, Level4, AlgaeHigh, AlgaeLow
    }

    private final Drive driveSub;
    private final Vision visionSub;

    private ArmState actualState = ArmState.Home;
    public ArmState desiredState = ArmState.Home;

    private double desiredAngle = 0; // CHANGE PLACEHOLDER
    private double desiredHeight = 0;

    public boolean targetIsVisible = false;
    private boolean simulationInitialized = false;

    public Arm(Vision visionSub, Drive driveSub) {
        // bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        armMotor = new TalonFX(ElectronicsIDs.ArmDummyMotorID);
        armMotorSim = armMotor.getSimState();

        wristMotor = new SparkMax(1, MotorType.kBrushless);
        wristMotorSim = new SimDeviceSim("SPARK MAX [" + wristMotor.getDeviceId() + "]");

        wristEncoder = new CANcoder(ElectronicsIDs.EncoderDummyID, "rio");
        wristEncoderSim = wristEncoder.getSimState();
        
        wristPidController = wristMotor.getClosedLoopController();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorDummyMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();
        leftElevatorMotor.setPosition(0);

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorDummyMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState();

        applyAngleEncoderConfigs();
        setNeutralMode(NeutralModeValue.Brake);
        this.driveSub = driveSub;
        this.visionSub = visionSub;

        wristMotorConfig.closedLoop
            .pidf(ArmConstants.WristKP, ArmConstants.WristKI, ArmConstants.WristKD, ArmConstants.WristFF)
            .iZone(ArmConstants.WristIZone)
            .outputRange(-1, 1);        
    }

    private void setDesiredAngleAndHeight() {

        switch (desiredState) {
            case Home: 
                break;
            case LoadCoral: 
                break;
            case Level1:
                break;
            case Level2:
                break;
            case Level3:
                break;
            case Level4:
                break;
            case AlgaeHigh:
                break;
            case AlgaeLow:
                break;
        }
        //setAngle(desiredAngle);
        //setHeightInches(desiredHeight);
    }

    public boolean hasCompletedMovement() {
        return desiredState == actualState;
    }

    @Override
    public void periodic() {

        setDesiredAngleAndHeight();


        logData();

        // Shuffleboard.getTab("Match").add("Can See Tag", targetIsVisible);
        // Shuffleboard.getTab("Match").add("Desired Shooter Angle", desiredAngle);
    }

    public void setWristAngle(double desiredDegrees) {
        wristPidController.setReference(desiredDegrees, ControlType.kVelocity);
        // finish, doesn't work
    }

    public void stopArmMotor() {
        armMotor.set(0);
    }

    public double getAngleCANCoderDegrees() {
        return Units.rotationsToDegrees(wristEncoder.getAbsolutePosition().getValue().baseUnitMagnitude());
    }

    private double getTalonEncoderDegrees() {
        return Units.rotationsToDegrees(armMotor.getPosition().getValue().baseUnitMagnitude());
    }

    public void setElevatorHeightInches(double desiredInches) {
        double rotations = ((desiredInches - ArmConstants.StartingHeight) / 2)
        * ArmConstants.RotationsPerElevatorInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        leftElevatorMotor.setControl(request.withSlot(0));
    }

    public double getHeightInches() {
        double elevatorHeight = ((leftElevatorMotor.getPosition().getValue().baseUnitMagnitude()
                / ArmConstants.RotationsPerElevatorInch) * 2)
                + ArmConstants.StartingHeight;
        return elevatorHeight;
    }

    public void setBasePosition(double height) {
        armMotor.setPosition(height);
    }

    /* SHOOTER MOUNT STATES */

    public ArmState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(ArmState newState) {
        if (newState == ArmState.Home) {
            // this desired state is invalid and will be ignored
            return;
        }
        if (newState != desiredState) {
            actualState = ArmState.Home;
        }
        desiredState = newState;
    }

    public ArmState getArmState() {
        return actualState;
    }

    private void logData() {
        Logger.recordOutput("Arm/StateActual", actualState);
        Logger.recordOutput("Arm/StateDesired", desiredState);
    }

    /* CONFIG */

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

    private void setNeutralMode(NeutralModeValue armMotorMode) {
        armMotor.setNeutralMode(armMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        // They deprecated PhysicsSim...
        /* PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(wristMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001); */

        double encoderAngle = Units.degreesToRotations(this.desiredAngle);
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

       // TODO - Config Wrist Sim

        double currentAngle = getAngleCANCoderDegrees();
        double delta = desiredAngle - currentAngle;
        delta = Math.min(Math.abs(delta), 5.0) * Math.signum(delta);
        wristEncoder.setPosition(Units.degreesToRotations(currentAngle + delta));

       /*  // Only set the hall effect if we are moving down
        if (desiredHeight == ShooterMountConstants.FloorIntakeHeight && isWithinHeightTolerance()) {
            // The bottom hall effect returns false when at bottom and true otherwise
            bottomHallEffectSim.setValue(false);
        } else {
            bottomHallEffectSim.setValue(true);
        } */

    }
}
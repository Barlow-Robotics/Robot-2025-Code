// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;


// import static edu.wpi.first.units.Units.Amp;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

public class Arm extends SubsystemBase {

    TalonFX armMotor;
    private final DCMotorSim armMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 45),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState armMotorSim;
    private final CANcoder armEncoder; // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/FusedCANcoder/src/main/java/frc/robot/Robot.java
    private final SoftwareLimitSwitchConfigs armMotorLimitConfigs;


    private final Robot robot;

    private double desiredArmAngle = 90; 

    private boolean simulationInitialized = false;

    public Arm(Robot robot) {
        armMotor = new TalonFX(ElectronicsIDs.ArmMotorID);
        armMotorSim = armMotor.getSimState();
        armEncoder = new CANcoder(ElectronicsIDs.ArmEncoderID);
        armMotorLimitConfigs = new SoftwareLimitSwitchConfigs();

        applyAllConfigs();

        this.robot = robot;
    }


    public void setDesiredArmAngle( double angle) {
        desiredArmAngle = angle ;
    }

    private void setDesiredAnglesAndHeights() {
        setArmAngle(desiredArmAngle);
    }


    @Override
    public void periodic() {

        if (robot.isEnabled()) {
            setDesiredAnglesAndHeights();
        }

        // boundsCheck();

        logData();

        // BaseStatusSignal.refreshAll(
        //         armMotor.getFault_FusedSensorOutOfSync(false),
        //         armMotor.getStickyFault_FusedSensorOutOfSync(false),
        //         armMotor.getFault_RemoteSensorDataInvalid(false),
        //         armMotor.getStickyFault_RemoteSensorDataInvalid(false),
        //         armMotor.getPosition(false),
        //         armMotor.getVelocity(false),
        //         armEncoder.getPosition(false),
        //         armEncoder.getVelocity(false),
        //         armMotor.getRotorPosition(false));

    }


    public void setArmAngle(double desiredDegrees) {
        MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        request.EnableFOC = Constants.IsFocEnabled;
        armMotor.setControl(request);
    }


    // public boolean armHasCompletedMotion() {

    //     return isWithinArmAngleTolerance() ;

    //     // double targetAngle = desiredArmAngle;
    //     // double delta = Units.rotationsToDegrees(armMotor.getClosedLoopReference().getValue()) - targetAngle ;
    //     // Logger.recordOutput("Arm/ArmAngle/deltaToReference", delta);

    //     // if (Math.abs(delta) > 1.0) {
    //     //     return false;
    //     // }
    //     // return true;
    // }


    public double getArmEncoderDegrees() {
        return armEncoder.getAbsolutePosition().getValue().in(Degrees);
    }

    public double getArmTalonEncoderDegrees() {
        return armMotor.getPosition().getValue().in(Degrees);
    }


    // private boolean isAtDesiredState() {

    //     boolean result = armHasCompletedMotion() ;
    //     Logger.recordOutput("Arm/newElevatorHasCompletedMotion", result);        

    //     if (isWithinArmAngleTolerance() ) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }


    /* TOLERANCES */


    public boolean isWithinArmAngleTolerance() {
        boolean withinTolerance = (getArmEncoderDegrees() >= desiredArmAngle - ArmConstants.ArmAngleTolerance)
                && (getArmEncoderDegrees() <= desiredArmAngle + ArmConstants.ArmAngleTolerance);
        return withinTolerance;
    }


    // public boolean isAvailableToGoToReef() {
    //     ArmState desiredState = getDesiredState();
    //     // return (desiredState == ArmState.Running || desiredState == ArmState.Level1
    //     // || desiredState == ArmState.Level2
    //     // || desiredState == ArmState.Level3 || desiredState == ArmState.Level4
    //     // || desiredState == ArmState.PreLevel1 || desiredState == ArmState.PreLevel2
    //     // || desiredState == ArmState.PreLevel3 || desiredState == ArmState.PreLevel4
    //     // || desiredState == ArmState.AlgaeHigh || desiredState == ArmState.AlgaeLow);
    //     return (!(desiredState == ArmState.WaitingForCoral) && !(desiredState == ArmState.LoadCoral));
    // }

    // public boolean isAvailableToGoToCoralStation() {
    //     ArmState desiredState = getDesiredState();
    //     return (desiredState == ArmState.WaitingForCoral || desiredState == ArmState.LoadCoral);
    // }

    /* SAFETY */


    public void stopArmMotor() {
        armMotor.set(0);
    }

    /*
    Makes sure we never go past our limits of motion
    private void boundsCheck() {

        if ((getArmEncoderDegrees() <= ArmConstants.MinArmAngle
                && armMotor.getVelocity().getValueAsDouble() < 0
                && !isWithinArmAngleTolerance()) ||
                (getArmEncoderDegrees() >= ArmConstants.MaxArmAngle
                && armMotor.getVelocity().getValueAsDouble() > 0
                && !isWithinArmAngleTolerance())) { 
            stopArmMotor();
        }

    }
    */

    private void logData() {

        // Logger.recordOutput("Arm/HasCompletedMovment", armHasCompletedMotion());
        Logger.recordOutput("Arm/Tolerances/WithinArmTolerance", isWithinArmAngleTolerance());

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
        // Logger.recordOutput("Arm/ArmAngle/MoveentComplete", this.armHasCompletedMotion());

    }

    /* CONFIG */

    public void applyAllConfigs() {
        applyArmEncoderConfigs();
        applyArmMotorConfigs(InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake);

        armMotorLimitConfigs.withForwardSoftLimitEnable(true);
        armMotorLimitConfigs.withReverseSoftLimitEnable(true);
        armMotorLimitConfigs.withForwardSoftLimitThreshold(Units.degreesToRotations(ArmConstants.MaxArmAngle));
        armMotorLimitConfigs.withReverseSoftLimitThreshold(Units.degreesToRotations(ArmConstants.MinArmAngle));
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
        motor.resetSignalFrequencies() ;
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
        // armEncoder.optimizeBusUtilization() ;
    }


    private void setNeutralMode(NeutralModeValue armMotorMode) {
        armMotor.setNeutralMode(armMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(armMotor, 0.001);

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

    }

}

// (CHANGE) TODO: NEED TO SETUP RUNNING-BETWEEN POSITION
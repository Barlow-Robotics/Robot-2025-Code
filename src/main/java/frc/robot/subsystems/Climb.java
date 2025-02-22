// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsIDs;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */

    private TalonFX winchMotor;
    private final DCMotorSim winchMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared,
                    Constants.ClimbConstants.WinchMotorGearRatio), DCMotor.getKrakenX60Foc(1));
    TalonFXSimState winchMotorSim;

    private Servo servo;

    public Climb() {
        // Motor config
        winchMotor = new TalonFX(ElectronicsIDs.WinchMotorID);
        winchMotorSim = winchMotor.getSimState();
        winchMotor.setPosition(0);
        applyWinchMotorConfigs(InvertedValue.CounterClockwise_Positive); // May need 2 change this inversion

        // Servo conflig
        servo = new Servo(Constants.ElectronicsIDs.ServoID);
        servo.enableDeadbandElimination(true);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void stop() {
        servo.setSpeed(0);
        winchMotor.stopMotor();
    }

    public void latchOntoCage() {
        // CHANGE: do something with the servo here        
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(ClimbConstants.CageAngle));
        winchMotor.setControl(request);
    }

    public void windWinch() {
        // CHANGE: do something with the servo here   
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(ClimbConstants.WinchedAngle));
        winchMotor.setControl(request);
    }

    public double winchPos() {
        return winchMotor.getPosition().getValueAsDouble();
    }

    public void extendServo() {
        servo.setPosition(ClimbConstants.ServoExtendedPos); // Should double check that we actually want it fully extended and not at a specific value instead
    }

    public void retractServo() {
        servo.setPosition(0);
    }

    public double servoPos() {
        return servo.getPosition();
    }

    public boolean isLatchedOnCage() {
        boolean withinWinchTolerance = (winchPos() >= ClimbConstants.CageAngle - ClimbConstants.WinchTolerance) && (winchPos() <= ClimbConstants.CageAngle + ClimbConstants.WinchTolerance);
        boolean withinServoTolerance = (servoPos() >= ClimbConstants.ServoExtendedPos - ClimbConstants.ServoTolerance) && (servoPos() <= ClimbConstants.ServoExtendedPos + ClimbConstants.ServoTolerance);
        return withinWinchTolerance && withinServoTolerance;
    }

    public boolean isWinched() {
        boolean withinWinchTolerance = (winchPos() >= ClimbConstants.WinchedAngle - ClimbConstants.WinchTolerance) && (winchPos() <= ClimbConstants.WinchedAngle + ClimbConstants.WinchTolerance);
        boolean withinServoTolerance = (servoPos() >= 0 - ClimbConstants.ServoTolerance) && (servoPos() <= 0 + ClimbConstants.ServoTolerance);
        return withinWinchTolerance && withinServoTolerance;
    }

    public boolean withinTolerance(double trueVal, double desiredVal, double tolerance) {
        return (trueVal >= desiredVal - tolerance) && (trueVal <= desiredVal + tolerance);
    }

    private void applyWinchMotorConfigs(InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ClimbConstants.WinchKP;
        talonConfigs.Slot0.kI = ClimbConstants.WinchKI;
        talonConfigs.Slot0.kD = ClimbConstants.WinchKD;
        talonConfigs.Slot0.kV = ClimbConstants.WinchFF;
        talonConfigs.Slot0.kG = ClimbConstants.WinchKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.WinchCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.WinchAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimbConstants.WinchJerk;

        applyMotorConfigs(winchMotor, "winchMotor", talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs, InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK()) break; }
        if (!status.isOK()) System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());

        /* SET & APPLY INVERSION CONFIGS */
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = inversion;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK()) break; }
        if (!status.isOK()) System.out.println("Could not apply motor output configs to " + motor + " error code: " + status.toString());

        // /* SET & APPLY CURRENT LIMIT CONFIGS */
        // CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        // currentLimitConfigs.SupplyCurrentLimit = ClimbConstants.SupplyCurrentLimit;
        // currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // for (int i = 0; i < 5; ++i) {
        //     status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
        //     if (status.isOK()) break; }
        // if (!status.isOK()) System.out.println("Could not apply current limit configs to " + motor + " error code: " + status.toString());
    }

}

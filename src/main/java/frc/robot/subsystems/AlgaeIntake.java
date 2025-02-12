// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.sim.PhysicsSim;

public class AlgaeIntake extends SubsystemBase {

    TalonFXS liftMotor;
    TalonFXSSimState liftMotorSim;
    private final DCMotorSim liftMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1)); // DCMotor doesn't have a minion config - tune on real bot

    TalonFXS intakeMotor;
    TalonFXSSimState intakeMotorSim;
    private final DCMotorSim intakeMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1)); // DCMotor doesn't have a minion config - tune on real bot

    private double desiredLiftAngle = 0;
    private double desiredIntakeAngle = 0;

    private final Drive driveSub;
    private final Vision visionSub;

    private boolean simulationInitialized = false;
    private boolean isEjecting;

    public AlgaeIntake(Vision visionSub, Drive driveSub) {

        liftMotor = new TalonFXS(ElectronicsIDs.LiftMotorID);
        intakeMotor = new TalonFXS(ElectronicsIDs.AlgaeIntakeMotorID);

        liftMotorSim = liftMotor.getSimState();
        intakeMotorSim = intakeMotor.getSimState();
        
        liftMotor.setPosition(0);
        intakeMotor.setPosition(0);

        this.driveSub = driveSub;
        this.visionSub = visionSub;

        applyLiftMotorConfigs(InvertedValue.CounterClockwise_Positive);
        applyIntakeMotorConfigs(InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);
    }

    public void setLiftAngle(double desiredDegrees) {
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        liftMotor.setControl(request);
        desiredLiftAngle = desiredDegrees;
    }

    public void stopLiftMotor() {
        liftMotor.stopMotor();
    }

    public double getLiftTalonEncoderDegrees() {
        return liftMotor.getPosition().getValue().in(Degrees);
    }

    public double getIntakeTalonEncoderDegrees() {
        return intakeMotor.getPosition().getValue().in(Degrees);
    }

    public void startIntaking() {
        final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
        intakeMotor.setControl(request.withVelocity(AlgaeConstants.IntakeSpeed));
        isEjecting = false;
        desiredIntakeAngle = AlgaeConstants.IntakeSpeed;
    }

    public void startEjecting() {
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
        intakeMotor.setControl(m_request.withVelocity(AlgaeConstants.EjectSpeed));
        isEjecting = true;
        desiredIntakeAngle = AlgaeConstants.EjectSpeed;
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }
    
    /* TOLERANCES */
    private boolean isWithinLiftAngleTolerance() {
        boolean withinTolerance = (getLiftTalonEncoderDegrees() >= desiredLiftAngle - AlgaeConstants.LiftAngleTolerance)
                && (getLiftTalonEncoderDegrees() <= desiredLiftAngle + AlgaeConstants.LiftAngleTolerance);
        return withinTolerance;
    } 

    @Override
    public void periodic() {
        logData();
    }

    private void logData() {
        Logger.recordOutput("AlgaeIntake/LiftAngle/DegreesDesired", desiredLiftAngle);
        Logger.recordOutput("AlgaeIntake/LiftAngle/DegreesTalon", getLiftTalonEncoderDegrees());
        Logger.recordOutput("AlgaeIntake/LiftAngle/VoltageActual", liftMotor.getMotorVoltage().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/ClosedLoopError", liftMotor.getClosedLoopError().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/SupplyCurrent", liftMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/RPSActual", liftMotor.getVelocity().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/AccelerationActual", liftMotor.getAcceleration().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/WithinLiftAngleTolerance", isWithinLiftAngleTolerance());

        Logger.recordOutput("AlgaeIntake/IntakeSpeed/DegreesDesired", desiredIntakeAngle);
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/DegreesTalon", getIntakeTalonEncoderDegrees());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/VoltageActual", intakeMotor.getMotorVoltage().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/ClosedLoopError", intakeMotor.getClosedLoopError().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/SupplyCurrent", intakeMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/RPSActual", intakeMotor.getVelocity().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/AccelerationActual", intakeMotor.getAcceleration().getValue());

        Logger.recordOutput("AlgaeIntake/isEjecting", this.isEjecting);
        Logger.recordOutput("AlgaeIntake/isIntaking", !this.isEjecting);
        
    }

    private void applyLiftMotorConfigs(InvertedValue inversion) {
        TalonFXSConfiguration talonConfigs = new TalonFXSConfiguration();
        talonConfigs.Slot0.kP = AlgaeConstants.LiftKP;
        talonConfigs.Slot0.kI = AlgaeConstants.LiftKI;
        talonConfigs.Slot0.kD = AlgaeConstants.LiftKD;
        talonConfigs.Slot0.kV = AlgaeConstants.LiftFF;
        talonConfigs.Slot0.kG = AlgaeConstants.LiftKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = AlgaeConstants.LiftCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = AlgaeConstants.LiftAcceleration;
        motionMagicConfigs.MotionMagicJerk = AlgaeConstants.LiftJerk;

        applyMotorConfigs(liftMotor, "liftMotor", talonConfigs, inversion);
    }

    private void applyIntakeMotorConfigs(InvertedValue inversion) {
        TalonFXSConfiguration talonConfigs = new TalonFXSConfiguration();
        talonConfigs.Slot0.kP = AlgaeConstants.IntakeKP;
        talonConfigs.Slot0.kI = AlgaeConstants.IntakeKI;
        talonConfigs.Slot0.kD = AlgaeConstants.IntakeKD;
        talonConfigs.Slot0.kV = AlgaeConstants.IntakeFF;
        talonConfigs.Slot0.kG = AlgaeConstants.IntakeKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = AlgaeConstants.IntakeCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = AlgaeConstants.IntakeAcceleration;
        motionMagicConfigs.MotionMagicJerk = AlgaeConstants.IntakeJerk;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = talonConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = AlgaeConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true; // Start with stator limits off

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // Try five times to apply the Intake motor current config
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to intake motor, with error code: " + status.toString());
        }

        applyMotorConfigs(intakeMotor, "intakeMotor", talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFXS motor, String motorName, TalonFXSConfiguration configs,
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

    private void setNeutralMode(NeutralModeValue liftMotorMode, NeutralModeValue intakeMotorMode) {
        liftMotor.setNeutralMode(liftMotorMode);
        intakeMotor.setNeutralMode(intakeMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFXS(liftMotor, 0.001);
        PhysicsSim.getInstance().addTalonFXS(intakeMotor, 0.001);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        // Following pattern from:
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html

        intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double intakeVoltage = intakeMotorSim.getMotorVoltage();
        intakeMotorModel.setInputVoltage(intakeVoltage);
        intakeMotorModel.update(0.02);
        intakeMotorSim.setRotorVelocity(intakeMotorModel.getAngularVelocityRPM() / 60.0); // magic number?
        intakeMotorSim.setRawRotorPosition(intakeMotorModel.getAngularPositionRotations());

        liftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double liftVoltage = liftMotorSim.getMotorVoltage();
        liftMotorModel.setInputVoltage(liftVoltage);
        liftMotorModel.update(0.02); // magic number?
        liftMotorSim.setRotorVelocity(liftMotorModel.getAngularVelocityRPM() / 60.0); // magic number?
        liftMotorSim.setRawRotorPosition(liftMotorModel.getAngularPositionRotations());
    }
}
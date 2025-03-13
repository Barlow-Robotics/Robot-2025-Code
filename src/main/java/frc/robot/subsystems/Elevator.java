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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Acceleration;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;
// import frc.robot.subsystems.Gripper.GripperState;

public class Elevator extends SubsystemBase {

    TalonFX carriageMotor;
    private final DCMotorSim carriageMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState carriageMotorSim;
    private final SoftwareLimitSwitchConfigs carriageMotorLimitConfigs;

    TalonFX elevatorMotor;
    private final TalonFXSimState elevatorMotorSim;
    private final DCMotorSim elevatorMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1));
    private final SoftwareLimitSwitchConfigs elevatorMotorLimitConfigs;

    private final Robot robot;

    private double desiredCarriageHeight = 0;
    private double desiredCarriageSpeed = Constants.ArmConstants.CarriageCruiseVelocity ;
    private double lastCarriageSpeed = desiredCarriageSpeed ;
    private double desiredElevatorHeight = 0;
    private double desiredElevatorSpeed = Constants.ArmConstants.ElevatorCruiseVelocity ;
    private double lastElevatorSpeed = desiredElevatorSpeed ;

    private boolean simulationInitialized = false;


    public Elevator(Robot robot ) {

        carriageMotor = new TalonFX(ElectronicsIDs.CarriageMotorID);
        carriageMotorSim = carriageMotor.getSimState();
        carriageMotor.setPosition(0);

        elevatorMotor = new TalonFX(ElectronicsIDs.ElevatorMotorID);
        elevatorMotorSim = elevatorMotor.getSimState();
        elevatorMotor.setPosition(0);

        carriageMotorLimitConfigs = new SoftwareLimitSwitchConfigs();
        elevatorMotorLimitConfigs = new SoftwareLimitSwitchConfigs();

        applyAllConfigs();

        carriageMotor.setPosition(0.0) ;

        this.robot = robot;
    }


    public double getDesiredElevatorHeightInches() {
        return desiredElevatorHeight ;
    }


    public void setDesiredElevatorHeightInches(double height) {
        this.desiredElevatorHeight = height ;
    }
    

    public double getDesiredCarriageHeightInches() {
        return desiredCarriageHeight ;
    }
    public void setDesiredCarriageHeightInches(double height) {
        this.desiredCarriageHeight = height ;
    }

    private void setDesiredAnglesAndHeights() {
        setCarriageHeightInches(desiredCarriageHeight);
        setElevatorHeightInches(desiredElevatorHeight);
    }


    @Override
    public void periodic() {
        if (robot.isEnabled()) {
            setDesiredAnglesAndHeights();
        }

        logData();
    }


    public void setElevatorHeightInches(double desiredInches) {

        // if the desired motion magic velocity has changed, we need to update the configuration
        if (lastElevatorSpeed != desiredElevatorSpeed) {
            double rotationsPerSecond = desiredElevatorSpeed * ArmConstants.RotationsPerElevatorInch;
            double rotationsPerSecondPerSecond = rotationsPerSecond / 0.25;
            double jerk = rotationsPerSecondPerSecond / 0.1;
            applyMotionMagicConfigs(elevatorMotor, rotationsPerSecond, rotationsPerSecondPerSecond, jerk);
            lastElevatorSpeed = desiredElevatorSpeed ;
        }

        double rotations = ((desiredInches /*- ArmConstants.StartingElevatorHeight*/))
                * ArmConstants.RotationsPerElevatorInch;
        if ( (rotations / ArmConstants.RotationsPerElevatorInch) > 19 ) {
            int wpk = 1 ;
        }
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        request.EnableFOC = Constants.IsFocEnabled;
        elevatorMotor.setControl(request.withSlot(0));
    }


    private boolean elevatorHasCompletedMotion() {
        double targetRotations = desiredElevatorHeight * ArmConstants.RotationsPerElevatorInch; 
        double delta = elevatorMotor.getClosedLoopReference().getValue() - targetRotations ;           
        Logger.recordOutput("Elevator/ElevatorHeight/deltaToReference", delta);
        if ( Math.abs( delta ) > 0.5 ) {
            return false ;
        }
        return true ;
    }


    public double getElevatorHeightInches() {
        double elevatorHeight = ((elevatorMotor.getPosition().getValue().in(Rotations)
                / ArmConstants.RotationsPerElevatorInch));
        return elevatorHeight;
    }



    public void setCarriageHeightInches(double desiredInches) {

        // if the desired motion magic velocity has changed, we need to update the configuration
        if (lastCarriageSpeed != desiredCarriageSpeed) {
            double rotationsPerSecond = desiredCarriageSpeed * ArmConstants.RotationsPerCarriageInch;
            double rotationsPerSecondPerSecond = rotationsPerSecond / 0.25;
            double jerk = rotationsPerSecondPerSecond / 0.1;
            applyMotionMagicConfigs(carriageMotor, rotationsPerSecond, rotationsPerSecondPerSecond, jerk);
            lastCarriageSpeed = desiredCarriageSpeed ;
        }

        double rotations = desiredInches * ArmConstants.RotationsPerCarriageInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        request.EnableFOC = Constants.IsFocEnabled;
        carriageMotor.setControl(request.withSlot(0));
    }


    private boolean carriageHasCompletedMotion() {
        double targetRotations = desiredCarriageHeight * ArmConstants.RotationsPerCarriageInch;  
        double delta = carriageMotor.getClosedLoopReference().getValue() - targetRotations ; 
        Logger.recordOutput("Elevator/CarriageHeight/deltaToReference", delta);
         
        if ( Math.abs( delta ) > 0.5 ) {
            return false ;
        }
        return true ;
    }


    public double getCarriageHeightInches() {
        double carriageHeight = ((carriageMotor.getPosition().getValue().in(Rotations)
                / ArmConstants.RotationsPerCarriageInch));
        /* + ArmConstants.StartingCarriageHeight; */
        return carriageHeight;
    }

    /* ARM STATES */


    public boolean isAtDesiredHeight() {
        return elevatorHasCompletedMotion() && carriageHasCompletedMotion() ;
        // Logger.recordOutput("Arm/newElevatorHasCompletedMotion", result);        

        // if (isWithinElevatorHeightTolerance() && isWithinCarriageHeightTolerance()) {
        //     return true;
        // } else {
        //     return false;
        // }
    }

    /* TOLERANCES */

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


    /* SAFETY */

    public void stopElevatorMotor() {
        elevatorMotor.set(0);
    }

    public void stopCarriageMotor() {
        carriageMotor.stopMotor();
        carriageMotor.set(0);
    }

    public void resetCarriageEncoder() {
        carriageMotor.setPosition(0.0);
    }

    public void resetElevatorEncoder() {
        elevatorMotor.setPosition(0.0);
    }

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

    }

    private void logData() {

        Logger.recordOutput("Elevator/isAtDesiredHeight", isAtDesiredHeight());
        Logger.recordOutput("Elevator/Tolerances/WithinElevatorTolerance", isWithinElevatorHeightTolerance());
        Logger.recordOutput("Elevator/Tolerances/WithinCarriageTolerance", isWithinCarriageHeightTolerance());

        Logger.recordOutput("Elevator/ElevatorHeight/InchesDesired", desiredElevatorHeight);
        Logger.recordOutput("Elevator/ElevatorHeight/InchesActual", getElevatorHeightInches());
        Logger.recordOutput("Elevator/ElevatorHeight/VoltageActual", elevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/ClosedLoopError",
                elevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/ProportionalOutput",
                elevatorMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/DerivativeOutput",
                elevatorMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/IntegratedOutput",
                elevatorMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/SupplyCurrent", elevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/TempC", elevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/ControlMode", elevatorMotor.getControlMode().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/RotationsActual",
                elevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/ElevatorHeight/RotationsDesired",
                elevatorMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/RPSActual", elevatorMotor.getVelocity().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/AccelerationActual",
                elevatorMotor.getAcceleration().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/StatorCurrent", elevatorMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/ClosedLoopFF", elevatorMotor.getClosedLoopFeedForward().getValue());
        Logger.recordOutput("Elevator/ElevatorHeight/ClosedLoopReference", elevatorMotor.getClosedLoopReference().getValue()/ ArmConstants.RotationsPerElevatorInch);
        Logger.recordOutput("Elevator/ElevatorHeight/MotionMagicIsRunning", elevatorMotor.getMotionMagicIsRunning().getValue());


        Logger.recordOutput("Elevator/Carriage/InchesDesired", desiredCarriageHeight);
        Logger.recordOutput("Elevator/Carriage/InchesActual", getCarriageHeightInches());
        Logger.recordOutput("Elevator/Carriage/VoltageActual", carriageMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Carriage/ClosedLoopError", carriageMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Elevator/Carriage/ProportionalOutput",
                carriageMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Elevator/Carriage/DerivativeOutput",
                carriageMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Elevator/Carriage/IntegratedOutput",
                carriageMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Elevator/Carriage/SupplyCurrent", carriageMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Elevator/Carriage/TempC", carriageMotor.getDeviceTemp().getValue());
        Logger.recordOutput("Elevator/Carriage/ControlMode", carriageMotor.getControlMode().getValue());
        Logger.recordOutput("Elevator/Carriage/RotationsActual", carriageMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator/Carriage/RotationsDesired", carriageMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("Elevator/Carriage/RPSActual", carriageMotor.getVelocity().getValue());
        Logger.recordOutput("Elevator/Carriage/AccelerationActual", carriageMotor.getAcceleration().getValue());
        Logger.recordOutput("Elevator/Carriage/StatorCurrent", carriageMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Carriage/ClosedLoopOutput", carriageMotor.getClosedLoopOutput().getValue());
        Logger.recordOutput("Elevator/Carriage/ClosedLoopFF", carriageMotor.getClosedLoopFeedForward().getValue());
        Logger.recordOutput("Elevator/Carriage/ClosedLoopReference", carriageMotor.getClosedLoopReference().getValue()/ ArmConstants.RotationsPerCarriageInch);
        Logger.recordOutput("Elevator/Carriage/MotionMagicIsRunning", carriageMotor.getMotionMagicIsRunning().getValue());

        Logger.recordOutput("Elevator/Carriage/hasCompletedMotion", carriageHasCompletedMotion());
        Logger.recordOutput("Elevator/Elevator/hasCompletedMotion", elevatorHasCompletedMotion());

    }

    /* CONFIG */

    public void applyAllConfigs() {
        applyElevatorMotorConfigs(elevatorMotor, "elevatorMotor", InvertedValue.Clockwise_Positive);
        applyCarriageMotorConfigs(carriageMotor, "carriageMotor", InvertedValue.CounterClockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);

        elevatorMotorLimitConfigs.withForwardSoftLimitEnable(true);
        elevatorMotorLimitConfigs.withForwardSoftLimitThreshold(ArmConstants.RotationsPerElevatorInch*(ArmConstants.MaxElevatorHeight));
        carriageMotorLimitConfigs.withForwardSoftLimitEnable(true);
        carriageMotorLimitConfigs.withForwardSoftLimitThreshold(ArmConstants.RotationsPerCarriageInch*(ArmConstants.MaxCarriageHeight));
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

        talonConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;

        double rotationsPerSecond = ArmConstants.ElevatorCruiseVelocity
                * ArmConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        //WPK need to investigate this.
        double rotationsPerSecondPerSecond = (ArmConstants.ElevatorAcceleration
                * ArmConstants.RotationsPerElevatorInch) / 0.25;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecondPerSecond;

        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecondPerSecond / 0.1;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }


    private void applyCarriageMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ArmConstants.CarriageKP;
        talonConfigs.Slot0.kI = ArmConstants.CarriageKI;
        talonConfigs.Slot0.kD = ArmConstants.CarriageKD;
        talonConfigs.Slot0.kV = ArmConstants.CarriageKV;
        talonConfigs.Slot0.kG = ArmConstants.CarriageKG;
        talonConfigs.Slot0.kS = ArmConstants.CarriageKS;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;

        double rotationsPerSecond = ArmConstants.CarriageCruiseVelocity
                * ArmConstants.RotationsPerCarriageInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        //WPK need to investigate this.
        double rotationsPerSecondPerSecond = (ArmConstants.CarriageAcceleration
                * ArmConstants.RotationsPerCarriageInch) / 0.25;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecondPerSecond;

        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecondPerSecond / 0.1;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }




    // all units are relative to revolutions per second
    private void applyMotionMagicConfigs(TalonFX motor, double velocity, double accel, double jerk) {
        MotionMagicConfigs config = new MotionMagicConfigs() ;
        config.MotionMagicCruiseVelocity = velocity;
        config.MotionMagicAcceleration = accel;
        config.MotionMagicJerk = jerk;
        motor.getConfigurator().apply(config) ;
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


    private void setNeutralMode(NeutralModeValue elevatorMotorMode,
            NeutralModeValue carriageMotorMode) {
        elevatorMotor.setNeutralMode(elevatorMotorMode);
        carriageMotor.setNeutralMode(carriageMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(carriageMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(elevatorMotor, 0.001);

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

    }

    /** @param velocity in inches per second */
    public void setElevatorSpeed(Double velocity) {
        desiredElevatorSpeed = velocity ;
        VelocityVoltage request = new VelocityVoltage(desiredElevatorSpeed * ArmConstants.RotationsPerElevatorInch); // converts from inches/s to RPS
        elevatorMotor.setControl(request);
    }

    public double getElevatorSpeed() {
        return elevatorMotor.getVelocity().getValueAsDouble();
    }

    public double getElevatorCurrent() {
        return elevatorMotor.getSupplyCurrent().getValueAsDouble();
    }

    /** @param velocity in inches per second */
    public void setCarriageSpeed(Double velocity) {
        desiredCarriageSpeed = velocity ;
        VelocityVoltage request = new VelocityVoltage(desiredCarriageSpeed * ArmConstants.RotationsPerElevatorInch); // converts from inches/s to RPS
        carriageMotor.setControl(request);
    }

    public double getCarriageSpeed() {
        return carriageMotor.getVelocity().getValueAsDouble();
    }

    public double getCarriageCurrent() {
        return carriageMotor.getSupplyCurrent().getValueAsDouble();
    }
}

// (CHANGE) TODO: NEED TO SETUP RUNNING-BETWEEN POSITION
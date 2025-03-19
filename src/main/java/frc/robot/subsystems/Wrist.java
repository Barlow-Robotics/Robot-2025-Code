// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
// import frc.robot.subsystems.Gripper.GripperState;

public class Wrist extends SubsystemBase {

    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
    private final SparkMaxSim wristMotorSim;
    private final CANcoder wristEncoder;
    private final CANcoderSimState wristEncoderSim;
    private final ProfiledPIDController wristPIDController;
    private final SimpleMotorFeedforward feedForwardController ;


    private final DCMotorSim wristMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));

    private final Robot robot;

    private double desiredWristAngle = 0;

    private boolean simulationInitialized = false;

    public Wrist(Robot robot) {

        wristMotor = new SparkMax(ElectronicsIDs.WristMotorID, MotorType.kBrushless);
        wristMotorSim = new SparkMaxSim(wristMotor, DCMotor.getNeo550((1)));
        wristEncoder = new CANcoder(ElectronicsIDs.WristEncoderID, "rio");
        wristEncoderSim = wristEncoder.getSimState();

        applyAllConfigs();

        // wristPIDController = new ProfiledPIDController(5.0, 0.001, 0.2, new TrapezoidProfile.Constraints(
        //         ArmConstants.WristMaxAngularVelocity, ArmConstants.WristMaxAngularAcceleration));
        wristPIDController = new ProfiledPIDController(2.5, 0.000, 0.1, new TrapezoidProfile.Constraints(
                ArmConstants.WristMaxAngularVelocity, ArmConstants.WristMaxAngularAcceleration));
        wristPIDController.setIZone(Units.degreesToRotations(6.0));
        wristPIDController.setIntegratorRange(-0.5, 0.5) ;
        wristPIDController.setTolerance(Units.degreesToRotations(ArmConstants.WristAngleTolerance)) ;
        wristPIDController.enableContinuousInput(-Math.PI, Math.PI);

        feedForwardController = new SimpleMotorFeedforward(0.0, 1.15) ;

        this.robot = robot;
    }


    public void setDesiredAngle( double angle) {
        desiredWristAngle = angle ;
    }

    private void setDesiredAnglesAndHeights() {
        setWristAngle(desiredWristAngle);
    }


    @Override
    public void periodic() {

        if (robot.isEnabled()) {
            setDesiredAnglesAndHeights();
        }

        logData();
    }

    public void setWristAngle(double desiredDegrees) {
        final double currentRotations = wristEncoder.getAbsolutePosition().getValueAsDouble();  
        var setPoint = wristPIDController.getSetpoint() ;
        final double wristOutput = wristPIDController.calculate(currentRotations, Units.degreesToRotations(desiredDegrees));
        final double feedForward = feedForwardController.calculate(setPoint.velocity) ;
        wristMotor.setVoltage(wristOutput + feedForward);
        Logger.recordOutput("Wrist/PIDOutput",wristOutput);        
        Logger.recordOutput("Wrist/feedForward",feedForward);        
    }


    public boolean wristHasCompletedMotion() {
        Logger.recordOutput("Wrist/wristHasCompletedMotion", wristPIDController.atGoal());        
        return wristPIDController.atGoal() ;
    }



    public double getWristEncoderDegrees() {
        return wristEncoder.getAbsolutePosition().getValue().in(Degrees);
    }



    // private boolean isAtDesiredState() {
    //     boolean result = wristHasCompletedMotion() ;
    //     return result ;
    // }


    /*
    private boolean isWithinWristAngleTolerance() {
        boolean withinTolerance = (getWristEncoderDegrees() >= desiredWristAngle - ArmConstants.WristAngleTolerance)
                && (getWristEncoderDegrees() <= desiredWristAngle + ArmConstants.WristAngleTolerance);
        return withinTolerance;
    }
    */

    public void stopWristMotor() {
        wristMotor.set(0);
    }


    private void logData() {

        Logger.recordOutput("Wrist/WristAngle/DegreesDesired", desiredWristAngle);
        Logger.recordOutput("Wrist/WristAngle/DegreesCANcoder", getWristEncoderDegrees());
        Logger.recordOutput("Wrist/WristAngle/RotationsCANcoder", wristEncoder.getAbsolutePosition().getValue());
        Logger.recordOutput("Wrist/WristAngle/PositionError", wristPIDController.getPositionError());
        Logger.recordOutput("Wrist/WristAngle/VelocityError", wristPIDController.getVelocityError());
        Logger.recordOutput("Wrist/WristAngle/AccumulatedError", wristPIDController.getAccumulatedError());
        Logger.recordOutput("Wrist/WristAngle/SetPointPosition", wristPIDController.getSetpoint().position);
        Logger.recordOutput("Wrist/WristAngle/SetPointVelocity", wristPIDController.getSetpoint().velocity);
        Logger.recordOutput("Wrist/WristAngle/GoalPosition", wristPIDController.getGoal().position);
        Logger.recordOutput("Wrist/WristAngle/GoalVelocity", wristPIDController.getGoal().velocity);

        if (Robot.isSimulation()) {
            Logger.recordOutput("Wrist/WristAngle/SimulatedPosition", wristMotorSim.getPosition());
        }
    }

    /* CONFIG */

    public void applyAllConfigs() {
        applyWristEncoderConfigs();
        wristMotorConfig.inverted(true);
        wristMotor.configure(wristMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

        // wristEncoder.optimizeBusUtilization() ;
    }


    /* SIMULATION */

    public void simulationInit() {
        double wristEncoderAngle = Units.degreesToRotations(desiredWristAngle);
        wristEncoderSim.setRawPosition(wristEncoderAngle);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        // Wrist Motor Sim

        wristMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
        wristMotorModel.setInputVoltage(wristMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        wristMotorModel.update(0.02);
        wristMotorSim.iterate(wristMotorModel.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), 0.02);
        wristEncoderSim.setVelocity(wristMotorModel.getAngularVelocityRadPerSec());
        wristEncoderSim.setRawPosition(wristMotorModel.getAngularPositionRotations());
    
    }

}


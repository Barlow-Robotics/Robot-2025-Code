// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// import static edu.wpi.first.units.Units.Amp;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElectronicsIDs;


public class AlgaeIntake extends SubsystemBase {

  SparkMax liftMotor;
  private final SparkMaxSim liftMotorSim;
  
  SparkMaxConfig liftMotorConfig = new SparkMaxConfig();
  private final DCMotorSim liftMotorModel;
    
  private final CANcoder liftEncoder; 
  private final CANcoderSimState liftEncoderSim;

  SparkMax intakeMotor;
  private final SparkMaxSim intakeMotorSim;
  
  SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  public final SparkClosedLoopController intakePidController;
  private final DCMotorSim intakeMotorModel;


  private double desiredLiftAngle = 0;
  private double desiredIntakeAngle = 0;

  private final Drive driveSub;
  private final Vision visionSub;

  private boolean simulationInitialized = false;

  public AlgaeIntake(Vision visionSub, Drive driveSub) {

    intakeMotor = new SparkMax(ElectronicsIDs.AlgaeIntakeMotorID, MotorType.kBrushless);
    intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNeo550((1)));
    liftMotorModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1), DCMotor.getNeo550(1));

    intakeMotorConfig = new SparkMaxConfig();
    intakePidController = intakeMotor.getClosedLoopController();
    intakeMotorModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));

    intakeMotorConfig.closedLoop
            .pidf(AlgaeConstants.IntakeKP, AlgaeConstants.IntakeKI, AlgaeConstants.IntakeKD, AlgaeConstants.IntakeFF)
            .iZone(AlgaeConstants.IntakeIZone)
            .outputRange(-1, 1); 


    liftMotor = new SparkMax(ElectronicsIDs.LiftMotorID, MotorType.kBrushless);
    liftMotorSim = new SparkMaxSim(liftMotor, DCMotor.getNeo550((1)));
    liftEncoder = new CANcoder(ElectronicsIDs.LiftEncoderID, "rio");
    liftEncoderSim = liftEncoder.getSimState();

    liftMotorConfig.closedLoop
            .pidf(AlgaeConstants.LiftKP, AlgaeConstants.LiftKI, AlgaeConstants.LiftKD, AlgaeConstants.LiftFF)
            .iZone(AlgaeConstants.LiftIZone)
            .outputRange(-1, 1); 
    
    this.driveSub = driveSub;
    this.visionSub = visionSub;
  }

  public void setLiftAngle(double desiredDegrees) {
    liftMotor.getClosedLoopController().setReference(desiredDegrees, ControlType.kPosition);
  }

  public void stopLiftMotor() {
    liftMotor.set(0);
}

  public double getLiftEncoderDegrees() {
    return Units.rotationsToDegrees(liftEncoder.getAbsolutePosition().getValue().baseUnitMagnitude());
  }

  public void startIntaking() {
    intakePidController.setReference(AlgaeConstants.IntakeSpeed, ControlType.kVelocity);
  }

  public void startEjecting() {
    intakePidController.setReference(AlgaeConstants.EjectSpeed, ControlType.kVelocity);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    logData();
  }

  private void logData() {
    // put logging in here
  }

  /* SIMULATION */

  public void simulationInit() {

  }
  @Override
  public void simulationPeriodic() {
      if (!simulationInitialized) {
          simulationInit();
          simulationInitialized = true;
      }
      intakeMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
      double intakeVoltage = intakeMotorSim.getBusVoltage();
      intakeMotorModel.setInputVoltage(intakeVoltage);
      intakeMotorModel.update(0.02);
      intakeMotorSim.setVelocity(intakeMotorModel.getAngularVelocityRPM() / 60.0);

      liftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
      double liftVoltage = liftMotorSim.getBusVoltage();
      liftMotorModel.setInputVoltage(liftVoltage);
      liftMotorModel.update(0.02);
      liftMotorSim.setVelocity(liftMotorModel.getAngularVelocityRPM() / 60.0);
      
      double currentLiftAngle = getLiftEncoderDegrees();
      double delta = desiredLiftAngle - currentLiftAngle;
      delta = Math.min(Math.abs(delta), 5.0) * Math.signum(delta);
      liftEncoder.setPosition(Units.degreesToRotations(currentLiftAngle + delta));
  
  }
  // NEED TO FIX: Can't figure out how to get this to return velocity -Ang
  // public void setSpeed(double speed) {
  //   intakeMotor.set(speed);
  // }
}
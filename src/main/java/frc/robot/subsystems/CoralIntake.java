// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElectronicsIDs;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

public class CoralIntake extends SubsystemBase {

  SparkMax intakeMotor;
  SparkMaxConfig intakeMotorConfig;
  SparkMaxSim intakeMotorSim;
  DCMotorSim intakeMotorModel;
  public final SparkClosedLoopController intakePidController;
  private boolean simulationInitialized = false;

  private final Drive driveSub;
  private final Vision visionSub;

  /** Creates a new Coral. */
  public CoralIntake(Vision visionSub, Drive driveSub) {
    intakeMotor = new SparkMax(ElectronicsIDs.CoralIntakeMotorID, MotorType.kBrushless);

    intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNeo550((1)));
    intakeMotorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1), DCMotor.getNeo550(1));
    intakePidController = intakeMotor.getClosedLoopController();
    intakeMotorConfig.closedLoop
            .pidf(CoralConstants.IntakeKP, CoralConstants.IntakeKI, CoralConstants.IntakeKD, CoralConstants.IntakeFF)
            .iZone(CoralConstants.IntakeIZone)
            .outputRange(-1, 1); 
    this.driveSub = driveSub;
    this.visionSub = visionSub;
  }

  @Override
  public void periodic() {
    logData();
  }

  public void startIntaking() {
    intakeMotor.getClosedLoopController().setReference(CoralConstants.IntakeSpeed, ControlType.kVelocity);
  }

  public void startEjecting() {
    // NEED TO FIX // should be differents speed (need to make new constant)
    intakeMotor.getClosedLoopController().setReference(CoralConstants.EjectSpeed, ControlType.kVelocity);
  }
  public double getIntakeMotorDegrees() {
    return Units.rotationsToDegrees(intakeMotor.getAbsoluteEncoder().getPosition());
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
 /* LOGGING */
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
      
      //double currentLiftAngle = getLiftEncoderDegrees();
      //double delta = desiredLiftAngle - currentLiftAngle;
      //delta = Math.min(Math.abs(delta), 5.0) * Math.signum(delta);
      //liftEncoder.setPosition(Units.degreesToRotations(currentLiftAngle + delta));
  
  }
  // NEED TO FIX: Can't figure out how to get this to return velocity -Ang
  // public void setSpeed(double speed) {
  //   intakeMotor.set(speed);
  // }

  
}

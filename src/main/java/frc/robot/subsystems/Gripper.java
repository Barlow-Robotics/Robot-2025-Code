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
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElectronicsIDs;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

public class Gripper extends SubsystemBase {

  SparkMax gripperMotor;
  SparkMaxConfig gripperMotorConfig;
  SparkMaxSim gripperMotorSim;
  DCMotorSim gripperMotorModel;
  public final SparkClosedLoopController gripperPidController;
  private boolean simulationInitialized = false;
  private boolean isEjecting;

  private final Drive driveSub;
  private final Vision visionSub;

  /** Creates a new Coral. */
  public Gripper(Vision visionSub, Drive driveSub) {
    gripperMotor = new SparkMax(ElectronicsIDs.GripperMotorID, MotorType.kBrushless);

    gripperMotorSim = new SparkMaxSim(gripperMotor, DCMotor.getNeo550((1)));
    gripperMotorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1), DCMotor.getNeo550(1));
    gripperPidController = gripperMotor.getClosedLoopController();
    gripperMotorConfig = new SparkMaxConfig();
    gripperMotorConfig.closedLoop
            .pidf(CoralConstants.GripperKP, CoralConstants.GripperKI, CoralConstants.GripperKD, CoralConstants.GripperFF)
            .iZone(CoralConstants.GripperIZone)
            .outputRange(-1, 1); 
    this.driveSub = driveSub;
    this.visionSub = visionSub;
  }

  @Override
  public void periodic() {

    logData();
  }

  public void setVelocity(double speed) {
    gripperMotor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
    isEjecting = false;
  }

  // public void startEjecting() {
  //   // NEED TO FIX // should be differents speed (need to make new constant)
  //   gripperMotor.getClosedLoopController().setReference(CoralConstants.EjectSpeed, ControlType.kVelocity);
  //   isEjecting = true;
  // }
  
  public double getIntakeEncoderDegrees() {
    return Units.rotationsToDegrees(gripperMotor.getAbsoluteEncoder().getPosition());
  }

  public void stop() {
    gripperMotor.stopMotor();
  }
 /* LOGGING */
  private void logData() {
    Logger.recordOutput("Gripper/GripperMotor/DegreesCANCoder", getIntakeEncoderDegrees());
    Logger.recordOutput("Gripper/GripperMotor/RotationsCANCoder", gripperMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Gripper/GripperMotor/VoltageActual", gripperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Gripper/GripperMotor/RPSActual", gripperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Gripper/isEjecting", this.isEjecting);
    Logger.recordOutput("Gripper/isIntaking", !this.isEjecting);

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
      gripperMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
      double intakeVoltage = gripperMotorSim.getBusVoltage();
      gripperMotorModel.setInputVoltage(intakeVoltage);
      gripperMotorModel.update(0.02);
      gripperMotorSim.setVelocity(gripperMotorModel.getAngularVelocityRPM() / 60.0);
      
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

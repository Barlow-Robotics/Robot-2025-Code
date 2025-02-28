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
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.ElectronicsIDs;

import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Gripper extends SubsystemBase {

  SparkMax gripperMotor;
  SparkMaxConfig gripperMotorConfigBrake;
  SparkMaxConfig gripperMotorConfigCoast;
  SparkMaxSim gripperMotorSim;
  DCMotorSim gripperMotorModel;
  public final SparkClosedLoopController gripperPidController;
  private boolean simulationInitialized = false;
  private boolean isEjecting;
  private boolean firstRelease = false;
  private int timerCount = 0;
  public enum GripperState {
    CarryingCoral, PlacingCoral, ReleasingL1, TakingInCoral, FinishedReleasingL1
  }
  GripperState gripperState = GripperState.CarryingCoral;
  /** Creates a new Coral. */
  public Gripper() {
    gripperMotor = new SparkMax(ElectronicsIDs.GripperMotorID, MotorType.kBrushless);

    gripperMotorSim = new SparkMaxSim(gripperMotor, DCMotor.getNeo550((1)));
    gripperMotorModel = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1), DCMotor.getNeo550(1));
    gripperPidController = gripperMotor.getClosedLoopController();
    gripperMotorConfigBrake = new SparkMaxConfig();
    gripperMotorConfigBrake.closedLoop
            .pidf(GripperConstants.GripperKP.get(), GripperConstants.GripperKI.get(), GripperConstants.GripperKD.get(), GripperConstants.GripperFF.get())
            .iZone(GripperConstants.GripperIZone.get())
            .outputRange(-1, 1);
    gripperMotorConfigBrake.idleMode(IdleMode.kCoast);

    gripperMotor.configure(gripperMotorConfigBrake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    if (detectedCoral()) {
      gripperState = GripperState.CarryingCoral;
    }
    else if (gripperState == GripperState.CarryingCoral) {
      setBreakMode();
      firstRelease = true;
    }
    else if (gripperState == GripperState.PlacingCoral) {
      setCoastMode();
      firstRelease = true;
    }
    else if (gripperState == GripperState.ReleasingL1 && firstRelease) {
      startEjecting();
      firstRelease = false;
      timerCount++;
    }
    else if (gripperState == GripperState.ReleasingL1) {
      timerCount++;
    }
    else if (gripperState == GripperState.TakingInCoral) {
      firstRelease = true;
    }
    else {
      setBreakMode();
      firstRelease = true;
    }
    logData();

    if (timerCount == 30) {
      timerCount = 0;
      gripperState = GripperState.FinishedReleasingL1;
    }
  }

  public void setState(GripperState newState) {
    this.gripperState = newState;
  }


  public void setCoastMode() {
    gripperMotor.setVoltage(0);
  }
  public void setBreakMode() {
    gripperMotor.setVoltage(0.5);
  }

  public void setVelocity(double speed) {
    gripperMotor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
    gripperState = GripperState.TakingInCoral;
    isEjecting = false;
  }

  public GripperState getState() {
    return this.gripperState;
  }
  public void setVoltage(double voltage) {
    gripperMotor.getClosedLoopController().setReference(voltage, ControlType.kVoltage);
  }

  public boolean detectedCoral() {
    return (getCurrent() > Constants.GripperConstants.currentOfIntakedCoral.get());
  }

  public double getCurrent() {
    return gripperMotor.getOutputCurrent();
  }

  public void startEjecting() {
    gripperMotor.getClosedLoopController().setReference(Constants.GripperConstants.EjectSpeed.get(), ControlType.kVelocity);
    isEjecting = true;
  }
  
  public double getIntakeEncoderDegrees() {
    return Units.rotationsToDegrees(gripperMotor.getAbsoluteEncoder().getPosition());
  }

  public void stop() {
    gripperMotor.stopMotor();
  }
 /* LOGGING */
  private void logData() {
    Logger.recordOutput("Gripper/GripperMotor/DegreesCANcoder", getIntakeEncoderDegrees());
    Logger.recordOutput("Gripper/GripperMotor/RotationsCANCoder", gripperMotor.getAbsoluteEncoder().getPosition());
    Logger.recordOutput("Gripper/GripperMotor/VoltageActual", gripperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Gripper/GripperMotor/RPSActual", gripperMotor.getEncoder().getVelocity());
    Logger.recordOutput("Gripper/GripperMotor/StatorCurrent", gripperMotor.getOutputCurrent());

    Logger.recordOutput("Gripper/isEjecting", this.isEjecting);
    Logger.recordOutput("Gripper/isIntaking", !this.isEjecting);
    Logger.recordOutput("Gripper/GripperState", getState());

                          // Is there supposed to be an exclamation mark
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

  
}

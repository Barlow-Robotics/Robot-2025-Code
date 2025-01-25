// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj.simulation.DIOSim;
//import edu.wpi.first.wpilibj.simulation.ElevatorSim;
//import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;
//import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;


public class AlgaeIntake extends SubsystemBase {

  SparkMax liftMotor;
  private final SparkMaxSim liftMotorSim;
  SparkMaxConfig liftMotorConfig = new SparkMaxConfig();
  public final SparkClosedLoopController liftPidController;
  private final DCMotorSim liftMotorModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), Constants.jKgMetersSquared, 1), DCMotor.getNEO(1));
    
  private final CANcoder liftEncoder; 
  private final CANcoderSimState liftEncoderSim;

  private final Drive driveSub;
  private final Vision visionSub;

  private boolean simulationInitialized = false;

  public AlgaeIntake(Vision visionSub, Drive driveSub) {

    liftMotor = new SparkMax(ElectronicsIDs.WristMotorID, MotorType.kBrushless);
    liftMotorSim = new SparkMaxSim(liftMotor, DCMotor.getNeo550((1)));
    liftEncoder = new CANcoder(ElectronicsIDs.LiftEncoderID, "rio");
    liftEncoderSim = liftEncoder.getSimState();

    liftPidController = liftMotor.getClosedLoopController();

    liftMotorConfig.closedLoop
            .pidf(AlgaeConstants.LiftKP, AlgaeConstants.LiftKI, AlgaeConstants.LiftKD, AlgaeConstants.LiftFF)
            .iZone(AlgaeConstants.LiftIZone)
            .outputRange(-1, 1); 
    
    this.driveSub = driveSub;
    this.visionSub = visionSub;
  }

  public void setLiftAngle(double desiredDegrees) {
    liftPidController.setReference(desiredDegrees, ControlType.kPosition);
  }

  public void stopLiftMotor() {
    liftMotor.set(0);
}

  public double getLiftEncoderDegrees() {
    return Units.rotationsToDegrees(liftEncoder.getAbsolutePosition().getValue().baseUnitMagnitude());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void logData() {
    // put logging in here
  }

  // Can't figure out how to get this to return velocity, need to fix -Ang
  // public void setSpeed(double speed) {
  //   intakeMotor.set(speed);
  // }
}

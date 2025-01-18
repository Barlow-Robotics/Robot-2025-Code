// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  /* Creates a new Algae. */
  
  private SparkMax intakeMotor;


  public AlgaeIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getSpeed() {
    return intakeMotor.getEncoder().getVelocity();
  }

  // Can't figure out how to get this to return velocity, need to fix -Ang
  // public void setSpeed(double speed) {
  //   intakeMotor.set(speed);
  // }
}

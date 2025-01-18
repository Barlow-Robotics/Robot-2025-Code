// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  private TalonFX climbMotor; 

  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

  public double getSpeed() {
    return climbMotor.getVelocity().getValueAsDouble();
  }
  public void setSpeed(double speed) {
     climbMotor.set(speed);
  }

}

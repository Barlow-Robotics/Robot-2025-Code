// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {

  DigitalInput breakBeam ;
  /** Creates a new Chute. */
  public Chute() {
    breakBeam = new DigitalInput(5) ;
  }


  public boolean hasCoral() {
    return breakBeam.get() ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Chute/hasCoral", hasCoral());
  }
}

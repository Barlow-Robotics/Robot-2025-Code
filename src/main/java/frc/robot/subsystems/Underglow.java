// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Underglow extends SubsystemBase {
    /** Creates a new UnderGlow. */
    SerialPort port = null;

    int currentMode = 1;

    public Underglow() {
        try {
            port = new SerialPort(9600, Constants.UnderGlowConstants.Port);
        } catch (Exception e) {
            port = null;
        }
    }

    @Override
    public void periodic() {
        if (port == null) {
            try {
                port = new SerialPort(9600, Constants.UnderGlowConstants.Port);
            } catch (Exception e) {
                port = null;
            }
        }
        else {
            // BCW: Do an if statement here depending on if the coralIsLoaded or not. 
            int desiredMode;
            desiredMode = Constants.UnderGlowConstants.NeonGreen;
            desiredMode = Constants.UnderGlowConstants.CoralLoaded;


            port.write(new byte[] { (byte) desiredMode }, 1);
            Logger.recordOutput("Underglow/byte", ((byte) desiredMode));
                // } catch (Exception ex) {
                // Logger.recordOutput("Underglow/problem", ex.toString());
                // }
            currentMode = desiredMode;
            
            Logger.recordOutput("Underglow/desiredMode", desiredMode);
        }
    }
}
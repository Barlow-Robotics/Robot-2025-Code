// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
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
        } catch (Exception ex) {

        }
    }

    @Override
    public void periodic() {
        int desiredMode = Constants.UnderGlowConstants.NeonGreen;

        if (DriverStation.isEnabled()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                desiredMode = Constants.UnderGlowConstants.BlueAlliance;
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                desiredMode = Constants.UnderGlowConstants.RedAlliance;
            }
        } else {
            desiredMode = Constants.UnderGlowConstants.NeonGreen;
        }

        if (currentMode != desiredMode && port != null) {
            try {
                port.write(new byte[] { (byte) desiredMode }, 1);
                Logger.recordOutput("Underglow/byte", ((byte) desiredMode));
            } catch (Exception ex) {
                Logger.recordOutput("Underglow/problem", ex.toString());
            }
            currentMode = desiredMode;
        }
        Logger.recordOutput("Underglow/desiredMode", desiredMode);

    }
}
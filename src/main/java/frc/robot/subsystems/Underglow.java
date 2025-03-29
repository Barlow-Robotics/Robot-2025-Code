// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climb.ClimbState;

public class Underglow extends SubsystemBase {
    /** Creates a new UnderGlow. */
    SerialPort port = null;
    int retryCount = 0 ;

    // int currentMode = 1;

    Robot robot ;
    Chute chute ;
    Climb climb ;

    public Underglow(Robot r, Chute c, Climb cl) {
        robot = r ;
        chute = c ;
        climb = cl ;
        try {
            port = new SerialPort(9600, Constants.UnderGlowConstants.Port);
        } catch (Exception e) {
            port = null;
        }
    }

    @Override
    public void periodic() {
        if (port == null) {
            // try to connect if we're disabled. Retry only once a second.
            if (!robot.isEnabled() && retryCount == 0) {
                try {
                    // port = new SerialPort(9600, Constants.UnderGlowConstants.Port);
                } catch (Exception e) {
                    port = null;
                }
            } else {
                retryCount = (retryCount +1) % 50 ;
            }
        } else {
            // BCW: Do an if statement here depending on if the coralIsLoaded or not. 
            int desiredMode;
            if ( climb.getCurrentState() == ClimbState.ReadyToLatch) {
                desiredMode = Constants.UnderGlowConstants.ClimbReadyToLatch ;
            }else if ( chute.hasCoral()) {
                desiredMode = Constants.UnderGlowConstants.CoralLoaded;
            } else {
                desiredMode = Constants.UnderGlowConstants.NeonGreen;
            }
            port.write(new byte[] { (byte) desiredMode }, 1);
            // Logger.recordOutput("Underglow/byte", ((byte) desiredMode));
            // currentMode = desiredMode;
           
            Logger.recordOutput("Underglow/desiredMode", desiredMode);
        }
    }
}
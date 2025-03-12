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

import com.revrobotics.sim.SparkMaxSim;
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

    private boolean simulationInitialized = false;
    private boolean intaking = false ;

    private boolean coralLoaded = false ;
    // private boolean isEjecting;
    // private boolean firstRelease = false;
    // private int timerCount = 0;

    private enum GripperState {
        // CarryingCoral, PlacingCoral, ReleasingL1, TakingInCoral, FinishedReleasingL1
        CarryingCoral, TakingInCoral, NoCoral
    }

    GripperState gripperState = GripperState.NoCoral;

    /** Creates a new Coral. */
    public Gripper() {
        gripperMotor = new SparkMax(ElectronicsIDs.GripperMotorID, MotorType.kBrushless);

        gripperMotorSim = new SparkMaxSim(gripperMotor, DCMotor.getNeo550((1)));
        gripperMotorModel = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), Constants.jKgMetersSquared, 1),
                DCMotor.getNeo550(1));
        // gripperPidController = gripperMotor.getClosedLoopController();
        gripperMotorConfigBrake = new SparkMaxConfig();
        gripperMotorConfigBrake.closedLoop
                .pidf(GripperConstants.GripperKP2, GripperConstants.GripperKI2, GripperConstants.GripperKD2,
                        GripperConstants.GripperFF2)
                .iZone(GripperConstants.GripperIZone.get())
                .outputRange(-1, 1);
        gripperMotorConfigBrake.idleMode(IdleMode.kCoast);

        gripperMotor.configure(gripperMotorConfigBrake, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }


    @Override
    public void periodic() {

        // wpk change over to velocity control when we have time

        if ( gripperState == GripperState.CarryingCoral || gripperState == GripperState.TakingInCoral ) {
            if (Math.abs(gripperMotor.getEncoder().getVelocity()) < 5 ) {
                gripperState = GripperState.CarryingCoral ;
            }
        }

        logData();

        // if (detectedCoral()) {
        //     gripperState = GripperState.CarryingCoral;
        // } else if (gripperState == GripperState.CarryingCoral) {
        //     setBreakMode();
        //     firstRelease = true;
        // } else if (gripperState == GripperState.PlacingCoral) {
        //     setCoastMode();
        //     firstRelease = true;
        // } else if (gripperState == GripperState.ReleasingL1 && firstRelease) {
        //     startEjecting();
        //     firstRelease = false;
        //     timerCount++;
        // } else if (gripperState == GripperState.ReleasingL1) {
        //     timerCount++;
        // } else if (gripperState == GripperState.TakingInCoral) {
        //     firstRelease = true;
        // } else {
        //     setBreakMode();
        //     firstRelease = true;
        // }

        // if (timerCount == 30) {
        //     timerCount = 0;
        //     gripperState = GripperState.FinishedReleasingL1;
        // }
    }

    // public void setState(GripperState newState) {
    //     this.gripperState = newState;
    // }

    public void setReleaseMode() {
        gripperMotor.setVoltage(0);
    }

    public void setHoldMode() {
        gripperMotor.setVoltage(0.25);
    }

    public void startIntaking() {
        gripperMotor.setVoltage(3);
        gripperState = GripperState.TakingInCoral ;
    }

    public void startEjecting() {
        gripperMotor.setVoltage(-2.0);
        gripperState = GripperState.NoCoral ;
        // gripperMotor.getClosedLoopController().setReference(Constants.GripperConstants.EjectSpeed2,
        // ControlType.kVelocity);

        // isEjecting = true;
    }

    public void releaseCoral() {
        gripperMotor.setVoltage(-1.0);
        gripperState = GripperState.NoCoral ;
    }

    public void startAlgaeRemoval() {
        gripperMotor.setVoltage(-9);
        gripperState = GripperState.NoCoral ;

    }

    // public void setVelocity(double speed) {
    //     gripperMotor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
    //     gripperState = GripperState.TakingInCoral;
    //     isEjecting = false;
    // }

    public void stop() {
        gripperMotor.stopMotor();
    }


    // public GripperState getState() {
    //     return this.gripperState;
    // }

    // public void setVoltage(double voltage) {
    //     gripperMotor.getClosedLoopController().setReference(voltage, ControlType.kVoltage);
    // }

    public boolean hasCoral() {
        // return (gripperState == GripperState.CarryingCoral);
        return false;
    }

    private double getCurrent() {
        return gripperMotor.getOutputCurrent();
    }


    private double getIntakeEncoderDegrees() {
        return Units.rotationsToDegrees(gripperMotor.getAbsoluteEncoder().getPosition());
    }

    /* LOGGING */
    private void logData() {
        Logger.recordOutput("Gripper/GripperMotor/RotationsCANCoder", gripperMotor.getAbsoluteEncoder().getPosition());
        // Logger.recordOutput("Gripper/GripperMotor/RotationsCANCoder", gripperMotor);
        Logger.recordOutput("Gripper/GripperMotor/VoltageActual", gripperMotor.getBusVoltage() );
        Logger.recordOutput("Gripper/GripperMotor/RPSActual", gripperMotor.getEncoder().getVelocity());
        Logger.recordOutput("Gripper/GripperMotor/StatorCurrent", gripperMotor.getOutputCurrent());

        // Logger.recordOutput("Gripper/GripperMotor/StatorCurrent",
        // gripperMotor.getClosedLoopController());

        // Logger.recordOutput("Gripper/firstRelease", this.firstRelease);
        // Logger.recordOutput("Gripper/isEjecting", this.isEjecting);
        // Logger.recordOutput("Gripper/isIntaking", !this.isEjecting);
        // // Logger.recordOutput("Gripper/GripperState", getState());

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

        // double currentLiftAngle = getLiftEncoderDegrees();
        // double delta = desiredLiftAngle - currentLiftAngle;
        // delta = Math.min(Math.abs(delta), 5.0) * Math.signum(delta);
        // liftEncoder.setPosition(Units.degreesToRotations(currentLiftAngle + delta));

    }

}

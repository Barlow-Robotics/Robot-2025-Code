// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class MotorUtils {


    public static void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        /* SET & APPLY INVERSION CONFIGS */

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT CONFIGS */

        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ArmConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motor + " error code: " + status.toString());
        }

        motor.optimizeBusUtilization() ;
    }




    public static void applyMotorConfigs(TalonFXS motor, String motorName, TalonFXSConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        /* SET & APPLY INVERSION CONFIGS */

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT CONFIGS */

        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ArmConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motor + " error code: " + status.toString());
        }

        motor.optimizeBusUtilization() ;
    }
    

}

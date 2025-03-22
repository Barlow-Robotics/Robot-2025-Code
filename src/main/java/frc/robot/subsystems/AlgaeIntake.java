// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.Vision.TargetToAlign;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.sim.PhysicsSim;

public class AlgaeIntake extends SubsystemBase {

    TalonFXS liftMotor;
    TalonFXSSimState liftMotorSim;
    private final DCMotorSim liftMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 4.28),
            DCMotor.getKrakenX60Foc(1)); // DCMotor doesn't have a minion config - tune on real bot

    TalonFXS intakeMotor;
    TalonFXSSimState intakeMotorSim;
    private final DCMotorSim intakeMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared, 1),
            DCMotor.getKrakenX60Foc(1)); // DCMotor doesn't have a minion config - tune on real bot

    double lastLiftAngle ; 
    LinearFilter deltaAngle = LinearFilter.movingAverage(10) ;

    // private double desiredLiftAngle = Constants.AlgaeConstants.retractedAngle.get() ;
    // private double desiredIntakeSpeed = 0;

    private boolean simulationInitialized = false;

    enum IntakeState { Retracted, Retracting, Deploying, Deployed, Receiving, HoldingAlgae, Ejecting } ;

    IntakeState currentState = IntakeState.Retracted ;


    private static final int DeploySlot = 0 ;
    private static final int RetractSlot = 1 ;
    private static final int HoldSlot = 2 ;


    public AlgaeIntake() {

        liftMotor = new TalonFXS(ElectronicsIDs.LiftMotorID);
        intakeMotor = new TalonFXS(ElectronicsIDs.AlgaeIntakeMotorID);

        liftMotorSim = liftMotor.getSimState();
        intakeMotorSim = intakeMotor.getSimState();
        
        applyLiftMotorConfigs(InvertedValue.Clockwise_Positive);
        applyIntakeMotorConfigs(InvertedValue.Clockwise_Positive);
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);

        double initialAngle = Constants.AlgaeConstants.LiftGearRatio *Units.degreesToRotations(Constants.AlgaeConstants.retractedAngle.get()) ;
        liftMotor.setPosition(initialAngle);
        lastLiftAngle = Constants.AlgaeConstants.retractedAngle.get() ;
    }

    public void deploy() {
        if ( currentState != IntakeState.Deployed && currentState != IntakeState.Deploying) {
            currentState = IntakeState.Deploying ;
        }

        // final double targetLiftRotations = Units.degreesToRotations(AlgaeConstants.deployedAngle2) * Constants.AlgaeConstants.LiftGearRatio ;
        // final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations);
        // liftMotor.setControl(liftRequest);

        // final VelocityVoltage intakeRequest = new VelocityVoltage(0);
        // intakeMotor.setControl(intakeRequest.withVelocity(AlgaeConstants.IntakeSpeed.get()));

        // desiredLiftAngle = AlgaeConstants.deployedAngle2;
        // desiredIntakeSpeed = AlgaeConstants.IntakeSpeed2;
    }
    
    public void retract() {
        if ( currentState != IntakeState.Retracted) {
            currentState = IntakeState.Retracting ;
        }

        // final double targetLiftRotations = Units.degreesToRotations(AlgaeConstants.restedAngle2) * Constants.AlgaeConstants.LiftGearRatio ;
        // final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations);
        // liftMotor.setControl(liftRequest);

        // final VelocityVoltage intakeRequest = new VelocityVoltage(0);
        // intakeMotor.setControl(intakeRequest.withVelocity(0));
        // desiredLiftAngle = AlgaeConstants.restedAngle2;
        // desiredIntakeSpeed = 0;
    }
    
    public void eject() {

        currentState = IntakeState.Ejecting ;
        // final MotionMagicVoltage liftRequest = new MotionMagicVoltage(Units.degreesToRotations(AlgaeConstants.restedAngle2));
        // liftMotor.setControl(liftRequest);

        // final VelocityVoltage intakeRequest = new VelocityVoltage(0);
        // intakeMotor.setControl(intakeRequest.withVelocity(AlgaeConstants.EjectSpeed.get()));
        // desiredLiftAngle = AlgaeConstants.restedAngle2;
        // desiredIntakeSpeed = AlgaeConstants.EjectSpeed.get();
    }

    public void stopLiftMotor() {
        liftMotor.stopMotor();
    }

    public double getLiftTalonEncoderDegrees() {
        return liftMotor.getPosition().getValue().in(Degrees) / Constants.AlgaeConstants.LiftGearRatio;
    }

    public double getIntakeTalonEncoderSpeed() {
        return intakeMotor.getVelocity().getValue().in(RotationsPerSecond);
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }
    
    /* TOLERANCES */


    private boolean intakeAngleIsDeployed() {
        return Math.abs(getLiftTalonEncoderDegrees() - AlgaeConstants.deployedAngle.get()) < AlgaeConstants.LiftAngleTolerance ;
    }

    private boolean intakeAngleIsRetracted() {
        return Math.abs(getLiftTalonEncoderDegrees() - AlgaeConstants.retractedAngle.get()) < AlgaeConstants.LiftAngleTolerance ;
    }



    // private boolean isWithinLiftAngleTolerance() {
    //     boolean withinTolerance = (getLiftTalonEncoderDegrees() >= desiredLiftAngle - AlgaeConstants.LiftAngleTolerance)
    //             && (getLiftTalonEncoderDegrees() <= desiredLiftAngle + AlgaeConstants.LiftAngleTolerance);
    //     return withinTolerance;
    // }

    // public boolean isDeployed() {
    //     if(getLiftTalonEncoderDegrees() >= (AlgaeConstants.deployedAngle.get() - 10)) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }


    // wpk magic numbers
    private boolean algaeIsBeingReceived(double angleTrend) {
        return (angleTrend < 1 && getLiftTalonEncoderDegrees() < AlgaeConstants.deployedAngle.get() - 10.0)
        && intakeMotor.getStatorCurrent().getValueAsDouble() > 50.0    ;     

    }


    @Override
    public void periodic() {

        double angleTrend = deltaAngle.calculate(getLiftTalonEncoderDegrees()-lastLiftAngle) ;
        Logger.recordOutput("AlgaeIntake/LiftAngle/AngleTrend", angleTrend);
        Logger.recordOutput("AlgaeIntake/LiftAngle/AlgeaBeingReceived", algaeIsBeingReceived(angleTrend));

        switch (currentState) {
            case Retracted: {
                // set motor control for lift to hold it in place
                double desiredPosition = Units.degreesToRotations(AlgaeConstants.retractedAngle.get())
                        * Constants.AlgaeConstants.LiftGearRatio;
                final PositionVoltage liftRequest = new PositionVoltage(desiredPosition).withSlot(HoldSlot);
                liftMotor.setControl(liftRequest);

                // stop roller motor
                // do we need to do something so algae doesn't roll out? Maybe set to position hold?
                final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);
                intakeMotor.setControl(intakeRequest);
            }
                break;

            case Retracting: {
                if (intakeAngleIsRetracted()) {
                    currentState = IntakeState.Retracted;
                } else {
                    // set lift motor control to lift
                    final double targetLiftRotations = Units.degreesToRotations(AlgaeConstants.retractedAngle.get())
                            * Constants.AlgaeConstants.LiftGearRatio;
                    final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations).withSlot(RetractSlot);
                    liftMotor.setControl(liftRequest);

                    // leave rollers turning incase we have a game piece
                    final VelocityVoltage intakeRequest = new VelocityVoltage(AlgaeConstants.IntakeSpeed.get()).withSlot(0);
                    intakeMotor.setControl(intakeRequest);
                }
            }
                break;

            case Deploying: {
                if (intakeAngleIsDeployed()) {
                    currentState = IntakeState.Deployed;
                } else {
                    // set lift motor control to deploy
                    final double targetLiftRotations = Units.degreesToRotations(AlgaeConstants.deployedAngle.get())
                            * Constants.AlgaeConstants.LiftGearRatio;
                    final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations).withSlot(DeploySlot);
                    liftMotor.setControl(liftRequest);

                    // set roller motors to capture speed
                    final VelocityVoltage intakeRequest = new VelocityVoltage(AlgaeConstants.IntakeSpeed.get()).withSlot(0);
                    intakeMotor.setControl(intakeRequest);

                    // wpk need to fix magic numbers
                    // check to see if we need to go to receiving state
                    // if the intake motors are drawing higher current
                    // and the delta position is heading toward the retracted state
                    if ( algaeIsBeingReceived(angleTrend)) {                    // if (angleTrend < 1 && getLiftTalonEncoderDegrees() < AlgaeConstants.deployedAngle.get() - 10.0) {
                        currentState = IntakeState.Receiving;
                    }
                }
            }
                break;

            case Deployed: {
                // set motor control for lift to hold it in place
                double desiredPosition = Units.degreesToRotations(AlgaeConstants.deployedAngle.get())
                        * Constants.AlgaeConstants.LiftGearRatio;
                final PositionVoltage liftRequest = new PositionVoltage(desiredPosition).withSlot(HoldSlot);
                liftMotor.setControl(liftRequest);

                // set roller motors to capture speed
                final VelocityVoltage intakeRequest = new VelocityVoltage(AlgaeConstants.IntakeSpeed.get()).withSlot(0);
                intakeMotor.setControl(intakeRequest);

                // wpk need to fix magic numbers
                // check to see if we need to go to receiving state
                // if the intake motors are drawing higher current
                // and the delta position is heading toward the retracted state
                if (algaeIsBeingReceived(angleTrend)) { // if (angleTrend < 1 && getLiftTalonEncoderDegrees() <
                                                        // AlgaeConstants.deployedAngle.get() - 10.0) {
                    currentState = IntakeState.Receiving;
                }
            }
                break;

            case Receiving: {

                // set motor control to let the intake rollers complete retracting the lift
                final VoltageOut liftRequest = new VoltageOut(0);
                liftMotor.setControl(liftRequest);

                // final double targetLiftRotations = Units.degreesToRotations(AlgaeConstants.retractedAngle.get())
                //         * Constants.AlgaeConstants.LiftGearRatio;
                // final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations)
                //         .withSlot(RetractSlot);
                // liftMotor.setControl(liftRequest);

                // set roller motors to capture speed
                final VelocityVoltage intakeRequest = new VelocityVoltage(AlgaeConstants.IntakeSpeed.get()).withSlot(0);
                intakeMotor.setControl(intakeRequest);

                // if we have the game piece
                if ( intakeAngleIsRetracted()) {
                    currentState = IntakeState.HoldingAlgae ;
                }

            }
                break;

            case HoldingAlgae: {

                // set motor control to let the intake rollers complete retracting the lift
                double desiredPosition = Units.degreesToRotations(AlgaeConstants.retractedAngle.get())
                        * Constants.AlgaeConstants.LiftGearRatio;
                final PositionVoltage liftRequest = new PositionVoltage(desiredPosition).withSlot(HoldSlot);
                liftMotor.setControl(liftRequest);

                // // set roller motors to capture speed
                // final VoltageOut intakeRequest = new VoltageOut(1.0);
                // intakeMotor.setControl(intakeRequest);

                final VelocityVoltage intakeRequest = new VelocityVoltage(0.5);
                intakeMotor.setControl(intakeRequest);


            }

                break;
    
            case Ejecting: {
                // leave lift motor control to what ever it was

                // set roller motors to eject speed
                final VelocityVoltage intakeRequest = new VelocityVoltage(AlgaeConstants.EjectSpeed.get()).withSlot(0);
                intakeMotor.setControl(intakeRequest);

            }
                break;

        }


        // final double targetLiftRotations = Units.degreesToRotations(desiredLiftAngle) * Constants.AlgaeConstants.LiftGearRatio ;
        // final MotionMagicVoltage liftRequest = new MotionMagicVoltage(targetLiftRotations);
        // liftMotor.setControl(liftRequest);

        logData();
        /* if(!isDeployed()) {
            stopIntakeMotor();
        } */

        lastLiftAngle = getLiftTalonEncoderDegrees() ;
    }

    private void logData() {
        // Logger.recordOutput("AlgaeIntake/LiftAngle/DegreesDesired", desiredLiftAngle);
        Logger.recordOutput("AlgaeIntake/LiftAngle/DegreesTalon", getLiftTalonEncoderDegrees());
        Logger.recordOutput("AlgaeIntake/LiftAngle/RotationsTalon",liftMotor.getPosition().getValue().in(Rotations)) ;
        Logger.recordOutput("AlgaeIntake/LiftAngle/VoltageActual", liftMotor.getMotorVoltage().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/ClosedLoopError", liftMotor.getClosedLoopError().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/ClosedLoopReference", 
                liftMotor.getClosedLoopReference().getValue() );
        Logger.recordOutput("AlgaeIntake/LiftAngle/ProportionalOutput", liftMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/DerivativeOutput", liftMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/IntegratedOutput", liftMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/Slot", liftMotor.getClosedLoopSlot().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/ClosedLoopFF", liftMotor.getClosedLoopFeedForward().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/SupplyCurrent", liftMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/StatorCurrent", liftMotor.getStatorCurrent().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/RPSActual", liftMotor.getVelocity().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/AccelerationActual", liftMotor.getAcceleration().getValue());
        Logger.recordOutput("AlgaeIntake/LiftAngle/intakeAngleIsDeployed", intakeAngleIsDeployed());
        Logger.recordOutput("AlgaeIntake/LiftAngle/intakeAngleIsRetracted", intakeAngleIsRetracted());

        Logger.recordOutput("AlgaeIntake/LiftAngle/StatorCurrent", liftMotor.getStatorCurrent().getValue());

        // Logger.recordOutput("AlgaeIntake/IntakeSpeed/SpeedDesired", desiredIntakeSpeed);
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/SpeedTalon", getIntakeTalonEncoderSpeed());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/VoltageActual", intakeMotor.getMotorVoltage().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/ClosedLoopError", intakeMotor.getClosedLoopError().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/ProportionalOutput", intakeMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/DerivativeOutput", intakeMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/IntegratedOutput", intakeMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/SupplyCurrent", intakeMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/RPSActual", intakeMotor.getVelocity().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeed/AccelerationActual", intakeMotor.getAcceleration().getValue());
        Logger.recordOutput("AlgaeIntake/IntakeSpeedHeight/StatorCurrent", intakeMotor.getStatorCurrent().getValue());

        Logger.recordOutput("AlgaeIntake/CurrentState", currentState);


        
    }

    private void applyLiftMotorConfigs(InvertedValue inversion) {
        TalonFXSConfiguration talonConfigs = new TalonFXSConfiguration();

        talonConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST ;

        // // Deploy gains
        // talonConfigs.Slot0.kP = 1.0;
        // talonConfigs.Slot0.kI = 0;
        // talonConfigs.Slot0.kD = 0.05;
        // talonConfigs.Slot0.kV = 4.0;
        // talonConfigs.Slot0.kG = 0.0;
        // talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // //retract gains
        // talonConfigs.Slot1.kP = 1.0;
        // talonConfigs.Slot1.kI = 0.0;
        // talonConfigs.Slot1.kD = 0.1;
        // talonConfigs.Slot1.kV = 6.0;
        // talonConfigs.Slot1.kG = 1.0;
        // talonConfigs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

        // // hold gains
        // talonConfigs.Slot2.kP = 6.0;
        // talonConfigs.Slot2.kI = 0.0;
        // talonConfigs.Slot2.kD = 0.0;
        // talonConfigs.Slot2.kV = 0;
        // talonConfigs.Slot2.kG = 0;
        // talonConfigs.Slot2.GravityType = GravityTypeValue.Arm_Cosine;


        // Deploy gains
        talonConfigs.Slot0.kP = 1.0;
        talonConfigs.Slot0.kI = 0;
        talonConfigs.Slot0.kD = 0.05;
        talonConfigs.Slot0.kV = 4.0;
        
        talonConfigs.Slot0.kG = 0.0;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        //retract gains
        talonConfigs.Slot1.kP = 2.0;
        talonConfigs.Slot1.kI = 0.0;
        talonConfigs.Slot1.kD = 0.1;
        talonConfigs.Slot1.kV = 8.0;
        talonConfigs.Slot1.kG = 1.0;
        talonConfigs.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

        // hold gains
        talonConfigs.Slot2.kP = 6.0;
        talonConfigs.Slot2.kI = 0.0;
        talonConfigs.Slot2.kD = 0.0;
        talonConfigs.Slot2.kV = 0;
        talonConfigs.Slot2.kG = 0;
        talonConfigs.Slot2.GravityType = GravityTypeValue.Arm_Cosine;





        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = AlgaeConstants.LiftCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = AlgaeConstants.LiftAcceleration;
        motionMagicConfigs.MotionMagicJerk = AlgaeConstants.LiftJerk;

        applyMotorConfigs(liftMotor, "liftMotor", talonConfigs, inversion);
        liftMotor.setNeutralMode(NeutralModeValue.Brake) ;
        //liftMotor.setPosition(null)
    }

    private void applyIntakeMotorConfigs(InvertedValue inversion) {
        TalonFXSConfiguration talonConfigs = new TalonFXSConfiguration();

        talonConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST ;

        talonConfigs.Slot0.kS = AlgaeConstants.IntakeKS.get();
        talonConfigs.Slot0.kV = AlgaeConstants.IntakeKV.get();
        talonConfigs.Slot0.kA = AlgaeConstants.IntakeKA.get();
        talonConfigs.Slot0.kP = AlgaeConstants.IntakeKP.get();
        talonConfigs.Slot0.kI = AlgaeConstants.IntakeKI.get();
        talonConfigs.Slot0.kD = AlgaeConstants.IntakeKD.get();
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;


        talonConfigs.Slot1.kS = 0.0;
        talonConfigs.Slot1.kV = 0.0;
        talonConfigs.Slot1.kA = 0.0;
        talonConfigs.Slot1.kP = 0.1;
        talonConfigs.Slot1.kI = 0.0;
        talonConfigs.Slot1.kD = 0.0;
        talonConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;


        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = AlgaeConstants.IntakeAcceleration;
        motionMagicConfigs.MotionMagicJerk = AlgaeConstants.IntakeJerk;

        // set current limit
        CurrentLimitsConfigs currentLimitConfigs = talonConfigs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = AlgaeConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true; // Start with stator limits off

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // Try five times to apply the Intake motor current config
        for (int i = 0; i < 5; ++i) {
            status = intakeMotor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to intake motor, with error code: " + status.toString());
        }

        applyMotorConfigs(intakeMotor, "intakeMotor", talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFXS motor, String motorName, TalonFXSConfiguration configs,
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

        motor.resetSignalFrequencies() ;

    }

    private void setNeutralMode(NeutralModeValue liftMotorMode, NeutralModeValue intakeMotorMode) {
        liftMotor.setNeutralMode(liftMotorMode);
        intakeMotor.setNeutralMode(intakeMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFXS(liftMotor, 0.001);
        PhysicsSim.getInstance().addTalonFXS(intakeMotor, 0.001);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        // Following pattern from:
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html

        intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double intakeVoltage = intakeMotorSim.getMotorVoltage();
        intakeMotorModel.setInputVoltage(intakeVoltage);
        intakeMotorModel.update(0.02);
        intakeMotorSim.setRotorVelocity(intakeMotorModel.getAngularVelocityRPM() / 60.0); // magic number?
        intakeMotorSim.setRawRotorPosition(intakeMotorModel.getAngularPositionRotations());

        liftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double liftVoltage = liftMotorSim.getMotorVoltage();
        liftMotorModel.setInputVoltage(liftVoltage);
        liftMotorModel.update(0.02); // magic number?
        liftMotorSim.setRotorVelocity(liftMotorModel.getAngularVelocityRPM() / 60.0); // magic number?
        liftMotorSim.setRawRotorPosition(liftMotorModel.getAngularPositionRotations());
    }
}
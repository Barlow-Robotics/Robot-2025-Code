// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.commands.TuneableParameter;
import frc.robot.sim.PhysicsSim;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */

    private TalonFX winchMotor;
    private final DCMotorSim winchMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared,
                    Constants.ClimbConstants.WinchMotorGearRatio), DCMotor.getKrakenX60Foc(1));
    TalonFXSimState winchMotorSim;
    public enum ClimbState {
        Default, LatchedOnCage, WinchedOnCage 
    }
    ClimbState currentState = ClimbState.Default;
    ClimbState desiredState = ClimbState.Default;
    private double desiredWinchAngle = 0;
    private double desiredgetServoPositionition = 0;
    private Servo servo;
    private boolean simulationInitialized = false;
    private int buttonStage = 0; // count presses

    public Climb() {
        // Motor config
        winchMotor = new TalonFX(ElectronicsIDs.WinchMotorID);
        winchMotorSim = winchMotor.getSimState();
        winchMotor.setPosition(0);
        applyWinchMotorConfigs(InvertedValue.CounterClockwise_Positive); // May need 2 change this inversion

        // Servo conflig
        servo = new Servo(Constants.ElectronicsIDs.ServoID);
        servo.enableDeadbandElimination(true);
    }

    @Override
    public void periodic() {
        if (isLatchedOnCage()) {
            currentState = ClimbState.LatchedOnCage;
            buttonStage = 1;
        }
        else if (isWinched()) {
            currentState = ClimbState.WinchedOnCage;
            buttonStage = 0;
        }
        else {
            currentState = ClimbState.Default;
        }
        logData();
    }

    public ClimbState getCurrentState() {
        return currentState;
    }

    public void stop() {
    //     servo.setSpeed(0);
        winchMotor.stopMotor();
    }

    public int returnButtonState() {
        return buttonStage; // use to enable two-tier climb function, wait for human player
    }

    public void latchOntoCage() {
        // Extend the servo here to disengage ratcheting
        //  CHANGE:  Do we need to wait or otherwise determine that the servo is fully extended before moving on?      
        extendServo();
        //  Once the servo is extended to release ratcheting, unwind the cable to move the harpoon forward
        //  IMPORTANT: 
        if (Math.abs(ClimbConstants.ServoExtendedPos - getServoPosition()) <= ClimbConstants.ServoTolerance)    // OLD CODE && (getServoPosition() <= (0 + ClimbConstants.ServoTolerance))
        {
            //  IMPORTANT: Comment out the following code until we know the servo is extending far enough to
            //      disengage ratcheting.
            final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(ClimbConstants.AttachAngle.get()));
            request.EnableFOC = Constants.IsFocEnabled;
            winchMotor.setControl(request);
            desiredWinchAngle = ClimbConstants.AttachAngle.get();   // Just for logging
        }
    }

    public void windWinch() {
        retractServo();
        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(ClimbConstants.ClimbedAngle.get()));
        request.EnableFOC = Constants.IsFocEnabled;
        winchMotor.setControl(request);
        desiredWinchAngle = ClimbConstants.ClimbedAngle.get();  //  Just for logging
    }

    public double getWinchPositionDegrees() {
        return winchMotor.getPosition().getValueAsDouble();
    }

    public void extendServo() {
        servo.setPosition(ClimbConstants.ServoExtendedPos);
        desiredgetServoPositionition = ClimbConstants.ServoExtendedPos;
    }

    public void retractServo() {
        servo.setPosition(ClimbConstants.ServoRetractedPos);
        desiredgetServoPositionition = ClimbConstants.ServoRetractedPos;
    }

    public double getServoPosition() {
        return servo.getPosition();
    }

    public boolean isLatchedOnCage() {
        boolean withinWinchTolerance = (getWinchPositionDegrees() >= ClimbConstants.AttachAngle.get() - ClimbConstants.WinchTolerance) && (getWinchPositionDegrees() <= ClimbConstants.AttachAngle.get() + ClimbConstants.WinchTolerance);
        boolean withinServoTolerance = (Math.abs(ClimbConstants.ServoExtendedPos - getServoPosition()) <= ClimbConstants.ServoTolerance);  // OLD CODE (getServoPosition() >= ClimbConstants.ServoExtendedPos - ClimbConstants.ServoTolerance) && (getServoPosition() <= ClimbConstants.ServoExtendedPos + ClimbConstants.ServoTolerance);
        return withinWinchTolerance && withinServoTolerance;
    }

    public boolean isWinched() {
        boolean withinWinchTolerance = (getWinchPositionDegrees() >= ClimbConstants.ClimbedAngle.get() - ClimbConstants.WinchTolerance) && (getWinchPositionDegrees() <= ClimbConstants.ClimbedAngle.get() + ClimbConstants.WinchTolerance);
        boolean withinServoTolerance = (getServoPosition() - Math.abs(ClimbConstants.ServoRetractedPos) <= ClimbConstants.ServoTolerance);
// OLD        boolean withinServoTolerance = (getServoPosition() >= 0 - ClimbConstants.ServoTolerance) && (getServoPosition() <= 0 + ClimbConstants.ServoTolerance);
        return withinWinchTolerance && withinServoTolerance;
    }

    //  Looks to be dead code.  Could use in above to methods.
    public boolean withinTolerance(double trueVal, double desiredVal, double tolerance) {
        return (trueVal >= desiredVal - tolerance) && (trueVal <= desiredVal + tolerance);
    }

    private void applyWinchMotorConfigs(InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ClimbConstants.WinchKP.get();
        talonConfigs.Slot0.kI = ClimbConstants.WinchKI.get();
        talonConfigs.Slot0.kD = ClimbConstants.WinchKD.get();
        talonConfigs.Slot0.kV = ClimbConstants.WinchFF.get();
        talonConfigs.Slot0.kG = ClimbConstants.WinchKG.get();
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.WinchCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.WinchAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimbConstants.WinchJerk;

        applyMotorConfigs(winchMotor, "winchMotor", talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs, InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK()) break; }
        if (!status.isOK()) System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());

        /* SET & APPLY INVERSION CONFIGS */
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = inversion;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK()) break; }
        if (!status.isOK()) System.out.println("Could not apply motor output configs to " + motor + " error code: " + status.toString());

        // /* SET & APPLY CURRENT LIMIT CONFIGS */
        // CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        // currentLimitConfigs.SupplyCurrentLimit = ClimbConstants.SupplyCurrentLimit;
        // currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // for (int i = 0; i < 5; ++i) {
        //     status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
        //     if (status.isOK()) break; }
        // if (!status.isOK()) System.out.println("Could not apply current limit configs to " + motor + " error code: " + status.toString());
    }

    
    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(winchMotor, 0.001);


        /* 
        elevatorHallEffectSim = new DIOSim(elevatorHallEffect);
        carriageHallEffectSim = new DIOSim(carriageHallEffect); */
    }

    @Override
    public void simulationPeriodic() {

        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    
        // Following pattern from:
        // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/simulation/simulation-intro.html
        // armMotorSim = armMotor.getSimState();
        // var carriageMotorSim = carriageMotor.getSimState();
        // Winch Motor Sim

        winchMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double winchVoltage = winchMotorSim.getMotorVoltage();
        winchMotorModel.setInputVoltage(winchVoltage);
        winchMotorModel.update(0.02);
        winchMotorSim.setRotorVelocity(winchMotorModel.getAngularVelocityRPM() / 60.0);
        winchMotorSim.setRawRotorPosition(winchMotorModel.getAngularPositionRotations());
       }    

    private void logData() {
        Logger.recordOutput("Climb/StateActual", currentState);
        Logger.recordOutput("Climb/StateDesired", desiredState);
    
        Logger.recordOutput("Climb/Winch/DegreesTalon", getWinchPositionDegrees());
        Logger.recordOutput("Climb/Winch/DegreesDesired", desiredWinchAngle);
        Logger.recordOutput("Climb/Winch/VoltageActual", winchMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Climb/Winch/ClosedLoopError", winchMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Climb/Winch/ProportionalOutput", winchMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Climb/Winch/DerivativeOutput", winchMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Climb/Winch/IntegratedOutput", winchMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Climb/Winch/ProportionalOutput", winchMotor.getClosedLoopProportionalOutput().getValue());
        Logger.recordOutput("Climb/Winch/DerivativeOutput", winchMotor.getClosedLoopDerivativeOutput().getValue());
        Logger.recordOutput("Climb/Winch/IntegratedOutput", winchMotor.getClosedLoopIntegratedOutput().getValue());
        Logger.recordOutput("Climb/Winch/SupplyCurrent", winchMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("Climb/Winch/StatorCurrent", winchMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Climb/Winch/RPSActual", winchMotor.getVelocity().getValue());
        Logger.recordOutput("Climb/Winch/AccelerationActual", winchMotor.getAcceleration().getValue());
        Logger.recordOutput("Climb/Winch/StatorCurrent", winchMotor.getStatorCurrent().getValue());

        Logger.recordOutput("Climb/Servo/PositionDesired", desiredgetServoPositionition);
        Logger.recordOutput("Climb/Servo/PositionActual", getServoPosition());
    }

}

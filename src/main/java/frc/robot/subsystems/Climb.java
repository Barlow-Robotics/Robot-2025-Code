// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

public class Climb extends SubsystemBase {
    /** Creates a new Climb. */

    Robot robot ;

    private TalonFX winchMotor;
    private final DCMotorSim winchMotorModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), Constants.jKgMetersSquared,
                    1),
            DCMotor.getKrakenX60Foc(1));
    TalonFXSimState winchMotorSim;

    public enum ClimbState {
        Idle, PrepareToUnwind, Unwind, ReadyToLatch, Wind
    }

    ClimbState currentState = ClimbState.Idle;
    // ClimbState desiredState = ClimbState.Idle;
    private double desiredWinchPosition = 0;
    private double desiredgetServoPositionition = 0;
    private Servo servo;
    private boolean simulationInitialized = false;

    private static final int WindGainSlot = 0 ;
    private static final int UnwindGainSlot = 1 ;

    private final BooleanSupplier releasePawlInTest;
    private final BooleanSupplier unwindWinchInTest; 
    private final BooleanSupplier windWinchInTest ;


    Timer timer ;

    public Climb(Robot r, BooleanSupplier releasePawlInTest, BooleanSupplier unwindWinchInTest, BooleanSupplier windWinchInTest) {
        robot = r ;

        // Motor config
        winchMotor = new TalonFX(ElectronicsIDs.WinchMotorID);
        winchMotorSim = winchMotor.getSimState();
        winchMotor.setPosition(0);
        applyWinchMotorConfigs(InvertedValue.CounterClockwise_Positive); // May need 2 change this inversion
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        // Servo conflig
        servo = new Servo(Constants.ElectronicsIDs.ServoID);
        servo.enableDeadbandElimination(true);
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        timer = new Timer() ;

        this.releasePawlInTest = releasePawlInTest ;
        this.unwindWinchInTest = unwindWinchInTest;
        this.windWinchInTest   = windWinchInTest;

    }

    @Override
    public void periodic() {

        if ( robot.isTeleopEnabled()) {
            switch (currentState) {
                case Idle: {
                    // don't do anything here
                }
                    break;
    
                case PrepareToUnwind: {
    
                    VoltageOut request = new VoltageOut(0.25);
                    request.EnableFOC = Constants.IsFocEnabled;
                    winchMotor.setControl(request);
                    releasePawl();
                    if (timer.get() > 0.2) {
                        timer.stop();
                        currentState = ClimbState.Unwind;
                    }
    
                }
                    break;
    
                case Unwind: {
                    MotionMagicVoltage request = new MotionMagicVoltage( ClimbConstants.WinchAttachRotations ) ;
                            // Units.degreesToRotations(ClimbConstants.UnwoundAngle.get()));
                    request.EnableFOC = Constants.IsFocEnabled;
                    winchMotor.setControl(request.withSlot(UnwindGainSlot));
                    desiredWinchPosition = ClimbConstants.WinchAttachRotations; // Just for logging
                    if (isUnwound()) {
                        winchMotor.set(0);
                        engagePawl();
                        currentState = ClimbState.ReadyToLatch;
                    }
                }
                    break;
    
                case ReadyToLatch: {
                    // don't do anything here. We're waiting for the driver to engage the cage
                    // and the operator to press the button
                }
                    break;
                case Wind: {
                    engagePawl();
                    // we want to get back to the position where we started before unwind which is zero because that's
                    // where it was when we started.
                    final MotionMagicVoltage request = new MotionMagicVoltage( 0 ) ;
                    request.EnableFOC = Constants.IsFocEnabled;
                    winchMotor.setControl(request.withSlot(WindGainSlot));
                    desiredWinchPosition = 0 ; // Just for logging
                    if (isWound()) {
                        winchMotor.set(0);
                        currentState = ClimbState.Idle;
                    }
                }
                    break;
            }
    
        } else if (robot.isTestEnabled()) {
            // logic to allow moving of winch during testing
            if (releasePawlInTest.getAsBoolean()) {
                releasePawl();
            } else {
                engagePawl();
            }

            if (unwindWinchInTest.getAsBoolean() && releasePawlInTest.getAsBoolean() ) {
                // set motor voltage to unwind
                final VoltageOut request = new VoltageOut( -6.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));
            } else if (windWinchInTest.getAsBoolean() ) {
                // set motor voltage to wind
                final VoltageOut request = new VoltageOut( 6.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));
            } else {
                // set motor voltage to wind
                final VoltageOut request = new VoltageOut( 0.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));
            }
        }

        logData();
    }

    public ClimbState getCurrentState() {
        return currentState;
    }

    public void goToUnwind() {
        releasePawl();
        timer.reset() ;
        timer.start() ;
        currentState = ClimbState.PrepareToUnwind ;
    }

    public void goToWind() {
        currentState = ClimbState.Wind;
    }

    public void stop() {
        winchMotor.stopMotor();
    }


    // public double getWinchPositionDegrees() {
    //     return winchMotor.getPosition().getValueAsDouble();
    // }

    public void releasePawl() {
        servo.setPosition(ClimbConstants.ServoExtendedPos); // extends the servo
        desiredgetServoPositionition = ClimbConstants.ServoExtendedPos;
    }

    public void engagePawl() {
        servo.setPosition(ClimbConstants.ServoRetractedPos); // retracts the servo
        desiredgetServoPositionition = ClimbConstants.ServoRetractedPos;
    }

    // public double getServoPosition() { // cant use this b/c it just return what its been set to, not what it's actually
    //                                    // at
    //     return servo.getPosition(); // will need to use a time based loop to get this working
    // }


    public boolean withinWinchTolerance(double rotations, double desiredRotations) {
        return Math.abs(rotations - desiredRotations) <= ClimbConstants.WinchTolerance;
    }

    public boolean isWound() {
        // wpk we should probably put that magic number in constants so we give it more meaning
        return withinWinchTolerance(winchMotor.getPosition().getValueAsDouble(), 0) ;
    }

    public boolean isUnwound() {
        // wpk we should probably put that magic number in constants so we give it more meaning
        boolean returnVal = withinWinchTolerance(winchMotor.getPosition().getValueAsDouble(), ClimbConstants.WinchAttachRotations) ;
        if ( returnVal){
            int wpk = 1;
        }
        return returnVal ;
    }




    // // Looks to be dead code. Could use in above to methods.
    // public boolean withinTolerance(double trueVal, double desiredVal, double tolerance) {
    //     return (trueVal >= desiredVal - tolerance) && (trueVal <= desiredVal + tolerance);
    // }

    private void applyWinchMotorConfigs(InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

        // gains for when we unwind the winch with the full weight of the robot
        // talonConfigs.Slot0.kP = ClimbConstants.WinchKP.get();
        // talonConfigs.Slot0.kI = ClimbConstants.WinchKI.get();
        // talonConfigs.Slot0.kD = ClimbConstants.WinchKD.get();
        // talonConfigs.Slot0.kV = ClimbConstants.WinchFF.get();
        // talonConfigs.Slot0.kG = ClimbConstants.WinchKG.get();
        talonConfigs.Slot0.kP = 0.3;
        talonConfigs.Slot0.kI = 0.0;
        talonConfigs.Slot0.kD = 0.0;
        talonConfigs.Slot0.kV = 0.15;
        talonConfigs.Slot0.kG = 0.0;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // gains for when we unwind the winch with no load attached
        talonConfigs.Slot1.kP = 0.1;
        talonConfigs.Slot1.kI = 0.0;
        talonConfigs.Slot1.kD = 0.0;
        talonConfigs.Slot1.kV = 0.15;
        talonConfigs.Slot1.kG = 0.0;
        talonConfigs.Slot1.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants.WinchCruiseRotationsPerSec;
        motionMagicConfigs.MotionMagicAcceleration = ClimbConstants.WinchAcceleration;
        motionMagicConfigs.MotionMagicJerk = ClimbConstants.WinchJerk;

        applyMotorConfigs(winchMotor, "winchMotor", talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());

        /* SET & APPLY INVERSION CONFIGS */
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = inversion;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK())
            System.out
                    .println("Could not apply motor output configs to " + motor + " error code: " + status.toString());

        // /* SET & APPLY CURRENT LIMIT CONFIGS */
        // CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        // currentLimitConfigs.SupplyCurrentLimit = ClimbConstants.SupplyCurrentLimit;
        // currentLimitConfigs.SupplyCurrentLimitEnable = true;
        // for (int i = 0; i < 5; ++i) {
        // status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
        // if (status.isOK()) break; }
        // if (!status.isOK()) System.out.println("Could not apply current limit configs
        // to " + motor + " error code: " + status.toString());

        motor.resetSignalFrequencies() ;

    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(winchMotor, 0.001);

        /*
         * elevatorHallEffectSim = new DIOSim(elevatorHallEffect);
         * carriageHallEffectSim = new DIOSim(carriageHallEffect);
         */
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
        winchMotorSim.setRotorVelocity(winchMotorModel.getAngularVelocity());
        winchMotorSim.setRawRotorPosition(winchMotorModel.getAngularPositionRotations());
    }

    private void logData() {
        Logger.recordOutput("Climb/StateActual", currentState);

        Logger.recordOutput("Climb/Winch/RotationsTalon", winchMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climb/Winch/RotationsDesired", desiredWinchPosition);
        Logger.recordOutput("Climb/Winch/VoltageActual", winchMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Climb/Winch/ClosedLoopReference", winchMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Climb/Winch/ClosedLoopFeedForward", winchMotor.getClosedLoopFeedForward().getValueAsDouble());
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

        Logger.recordOutput("Climb/Winch/Timer", timer.get());

        Logger.recordOutput("Climb/Servo/PositionDesired", desiredgetServoPositionition);
    }



    // wpk - these functions are only used in test mode to allow for easy rewind / unwind of the mechanism
    // when we need to reset it. It is assumed that there will be no robot attached to the winch so we'll use
    // the appropriate gain slot

    public void UnwindDuringTest() {
    }


    public void WindDuringTest() {
    }


}

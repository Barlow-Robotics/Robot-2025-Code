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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
        Idle, Unwind, ReadyToLatch, Wind
    }

    ClimbState currentState = ClimbState.Idle;

    private final DigitalInput hallSensor ;

    // ClimbState desiredState = ClimbState.Idle;
    private double desiredWinchPosition = 0;
    private boolean simulationInitialized = false;

    private final BooleanSupplier unwindWinchInTest; 
    private final BooleanSupplier windWinchInTest ;


    public Climb(Robot r, BooleanSupplier releasePawlInTest, BooleanSupplier unwindWinchInTest, BooleanSupplier windWinchInTest) {
        robot = r ;

        // Motor config
        winchMotor = new TalonFX(ElectronicsIDs.WinchMotorID);
        winchMotorSim = winchMotor.getSimState();
        winchMotor.setPosition(0);
        applyWinchMotorConfigs(InvertedValue.CounterClockwise_Positive); // May need 2 change this inversion
        winchMotor.setNeutralMode(NeutralModeValue.Brake);

        // Hall sensor conflig
        hallSensor = new DigitalInput(2) ;

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
    
                case Unwind: {
                    final VoltageOut request = new VoltageOut( -6.0 ) ;
                    winchMotor.setControl(request.withEnableFOC(true));
    
                    // MotionMagicVoltage request = new MotionMagicVoltage( ClimbConstants.WinchAttachRotations ) ;
                    //         // Units.degreesToRotations(ClimbConstants.UnwoundAngle.get()));
                    // request.EnableFOC = Constants.IsFocEnabled;
                    // winchMotor.setControl(request.withSlot(UnwindGainSlot));
                    desiredWinchPosition = ClimbConstants.WinchAttachRotations; // Just for logging
                    if (isUnwound()) {
                        winchMotor.set(0);
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
                    // we want to get back to the position where we started before unwind which is zero because that's
                    // where it was when we started.
                                    // set motor voltage to wind
                final VoltageOut request = new VoltageOut( 6.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));

                    // final MotionMagicVoltage request = new MotionMagicVoltage( 0 ) ;
                    // request.EnableFOC = Constants.IsFocEnabled;
                    // winchMotor.setControl(request.withSlot(WindGainSlot));
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

            if (unwindWinchInTest.getAsBoolean() ) {
                // set motor voltage to unwind
                final VoltageOut request = new VoltageOut( -6.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));
            } else if (windWinchInTest.getAsBoolean() && hallSensor.get() ) {
                // set motor voltage to wind
                final VoltageOut request = new VoltageOut( 6.0 ) ;
                winchMotor.setControl(request.withEnableFOC(true));
            } else {
                // set motor voltage to zero
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
        currentState = ClimbState.Unwind ;
    }

    public void goToWind() {
        currentState = ClimbState.Wind;
    }

    public void stop() {
        winchMotor.stopMotor();
    }



    public boolean withinWinchTolerance(double rotations, double desiredRotations) {
        return Math.abs(rotations - desiredRotations) <= ClimbConstants.WinchTolerance;
    }

    public boolean isWound() {
        // wpk we should probably put that magic number in constants so we give it more meaning

        boolean wound = withinWinchTolerance(winchMotor.getPosition().getValueAsDouble(), 0)
         || !hallSensor.get() ;

        return wound ;
    }

    public boolean isUnwound() {
        boolean returnVal = withinWinchTolerance(winchMotor.getPosition().getValueAsDouble(), ClimbConstants.WinchAttachRotations) ;
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
        Logger.recordOutput("Climb/Winch/hallSensor", hallSensor.get());
        Logger.recordOutput("Climb/Winch/isWound", isWound());

    }

}

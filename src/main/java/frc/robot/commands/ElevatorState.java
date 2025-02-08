package frc.robot.commands;

public class ElevatorState {
    private double elvatorHeight;
    private double carriageHeight;
    private double armAngle;
    private double wristAngle;
    private double intakeSpeed;
    public ElevatorState(double elvatorHeight, double carriageHeight, double armAngle, double wristAngle, double intakeSpeed) {
        this.elvatorHeight = elvatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.intakeSpeed = intakeSpeed;
    }
    public ElevatorState(double elvatorHeight, double carriageHeight, double armAngle, double wristAngle) {
        this.elvatorHeight = elvatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.intakeSpeed = 0;
    }

    public double getWristAngle() {
        return this.wristAngle;
    }

    public double getCarriageHeight() {
        return this.carriageHeight;
    }

    public double getArmAngle() {
        return this.armAngle;
    }
    
    public double getElevatorHeight() {
        return this.elvatorHeight;
    }

    public double getIntakeSpeed() {
        return this.intakeSpeed;
    }

}


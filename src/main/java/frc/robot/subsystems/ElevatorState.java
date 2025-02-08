package frc.robot.subsystems;

public class ElevatorState {
    private double elvatorHeight;
    private double carriageHeight;
    private double armAngle;
    private double wristAngle;
    private double gripperSpeed;
    public ElevatorState(double elvatorHeight, double carriageHeight, double armAngle, double wristAngle, double gripperSpeed) {
        this.elvatorHeight = elvatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.gripperSpeed = gripperSpeed;
    }
    public ElevatorState(double elvatorHeight, double carriageHeight, double armAngle, double wristAngle) {
        this.elvatorHeight = elvatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.gripperSpeed = 0;
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

    public double getGripperSpeed() {
        return this.gripperSpeed;
    }

}


package frc.robot.commands;

public class ArmStateParameters {
    private double elevatorHeight;
    private double carriageHeight;
    private double armAngle;
    private double wristAngle;
    private double gripperVelocity;

    public ArmStateParameters(double elevatorHeight, double carriageHeight, double armAngle, double wristAngle,
            double gripperVelocity) {
        this.elevatorHeight = elevatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.gripperVelocity = gripperVelocity;
    }

    public ArmStateParameters(double elevatorHeight, double carriageHeight, double armAngle, double wristAngle) {
        this.elevatorHeight = elevatorHeight;
        this.carriageHeight = carriageHeight;
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.gripperVelocity = 0;
    }

    public double getWristAngle() {
        return wristAngle;
    }

    public double getCarriageHeight() {
        return carriageHeight;
    }

    public double getArmAngle() {
        return armAngle;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }

    public double getGripperVelocity() {
        return gripperVelocity;
    }

}

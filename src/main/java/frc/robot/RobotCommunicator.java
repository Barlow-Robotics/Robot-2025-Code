// package frc.robot;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import java.awt.Point;

// public class RobotCommunicator {
//     private NetworkTableInstance ntInstance;
//     private NetworkTable table;

//     public RobotCommunicator() {
//         ntInstance = NetworkTableInstance.getDefault();
//         table = ntInstance.getTable("RobotCommands");
//     }

//     public void sendCoordinates(Point point) {
//         table.getEntry("gotoX").setDouble(point.x);
//         table.getEntry("gotoY").setDouble(point.y);
//     }
// }

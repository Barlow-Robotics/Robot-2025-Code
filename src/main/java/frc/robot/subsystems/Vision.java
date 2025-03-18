
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import static frc.robot.Constants.VisionConstants.FallbackVisionStrategy;
import static frc.robot.Constants.VisionConstants.FieldTagLayout;
import static frc.robot.Constants.VisionConstants.ClimbCameraName;
import static frc.robot.Constants.VisionConstants.ClimbCameraToRobot;
import static frc.robot.Constants.VisionConstants.PrimaryVisionStrategy;
import static frc.robot.Constants.VisionConstants.RightClimbCamName;
import static frc.robot.Constants.VisionConstants.RobotToElevatorCam;
import static frc.robot.Constants.VisionConstants.RobotToRightClimbCam;
import static frc.robot.Constants.VisionConstants.ElevatorCameraName;

import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    ////// ------ PHOTON VISION AND APRIL TAG VARIABLES ------ //////

    private PhotonCamera elevatorCamera;
    private PhotonCamera climbCamera;
    private PhotonCamera rightClimbCam;
    public final PhotonPoseEstimator ElevatorPhotonEstimator;
    public final PhotonPoseEstimator rightClimbPhotonEstimator;
    private double lastEstTimestamp = 0;
    private boolean disabledVision = false;
    private PhotonCameraSim climbCameraSim;
    private PhotonCameraSim elevatorCameraSim;
    private VisionSystemSim visionSim;
    private Transform3d robotToCamera;
    private PhotonTrackedTarget currentBestTarget;
    private PhotonTrackedTarget currentBestAlignTarget = null;
    public List<PhotonTrackedTarget> allDetectedTargets;
    private HashSet<Integer> targetAlignSet;
    public OptionalInt activeAlignTargetId;
    private Alliance alliance;
    private int pathRecounter = 0;
    boolean aprilTagDetected = false;

    // Hashtable<Integer, Integer> blueTrackableIDs = new Hashtable<>();
    // Hashtable<Integer, Integer> redTrackableIDs = new Hashtable<>();

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private boolean layoutOriginSet = false;

    private final Drive driveSub;
    private final Robot robot;
    ////// ------ CORAL VARIABLES ------ //////

    boolean coralDetected;
    double coralDistanceFromCenter;
    double coralHeight;
    double coralWidth;
    String sourceIP = "Nothing Received" ;

    private DatagramChannel visionChannel = null;
    ByteBuffer buffer = ByteBuffer.allocate(1024);

    public Vision(Drive driveSub, Robot robot) /* throws IOException */ {
        this.driveSub = driveSub;
        this.robot = robot;
        elevatorCamera = new PhotonCamera(ElevatorCameraName); // left camera
        rightClimbCam = new PhotonCamera(RightClimbCamName);
        climbCamera = new PhotonCamera(ClimbCameraName); // right camera

        // elevatorPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, poseCamera, PoseCameraToRobot);
        ElevatorPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, RobotToElevatorCam);
        ElevatorPhotonEstimator.setMultiTagFallbackStrategy(FallbackVisionStrategy);


        rightClimbPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, RobotToRightClimbCam);
        rightClimbPhotonEstimator.setMultiTagFallbackStrategy(FallbackVisionStrategy);

        alliance = DriverStation.Alliance.Red;
        if (DriverStation.isEnabled()) {
            var a = DriverStation.getAlliance();
            if (a.isPresent()) {
                alliance = a.get();
            }
        }

        targetAlignSet = new HashSet<Integer>();
        activeAlignTargetId = OptionalInt.empty();
        allDetectedTargets = new ArrayList<PhotonTrackedTarget>();

        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);


        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        this.aprilTagFieldLayout = layout;

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the
            // field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this
            // simulated field.
            visionSim.addAprilTags(FieldTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            climbCameraSim = new PhotonCameraSim(climbCamera, cameraProp);
            elevatorCameraSim = new PhotonCameraSim(elevatorCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.

            visionSim.addCamera(climbCameraSim, ClimbCameraToRobot);
            visionSim.addCamera(elevatorCameraSim, RobotToElevatorCam);

            climbCameraSim.enableDrawWireframe(true);
        }

        // catch (IOException e) {
        // // TODO decide what you want to do if the layout fails to load
        // elevatorPhotonEstimator = new PhotonPoseEstimator(null, null, camera, null);
        // }
        // Pose3d robotPose =
        // PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        // fieldTags.getTagPose(target.getFiducialId()), robotToCamera);
    }

    public AprilTagFieldLayout getLayout() {
        return this.aprilTagFieldLayout;
    }

    private void setLayoutOrigin() {
        if (!layoutOriginSet) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                aprilTagFieldLayout.setOrigin(
                        DriverStation.getAlliance().get() == Alliance.Blue
                                ? OriginPosition.kBlueAllianceWallRightSide
                                : OriginPosition.kRedAllianceWallRightSide);
                this.layoutOriginSet = true;
            }
        }
    }

    public void disableTheVision(boolean val) {
        this.disabledVision = val;
    }

    public void updateVisionLocalization(Pose2d drivePose) {
        var visionEst = getEstimatedGlobalPose(drivePose, VisionConstants.ElevatorCameraName);
        visionEst.ifPresent(
                est -> {
                    driveSub.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    Logger.recordOutput("Drive/ElevatorCameraPoseEstimate", est.estimatedPose.toPose2d());

                    // m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraOneStdDev);
                    // m_PoseEstimator.addVisionMeasurement(
                    //         est.estimatedPose.toPose2d(), est.timestampSeconds);
                });

        visionEst = getEstimatedGlobalPose(drivePose, VisionConstants.RightClimbCamName);
        visionEst.ifPresent(
                est -> {
                    // m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraTwoStdDev);
                    driveSub.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    Logger.recordOutput("Drive/RightClimbPoseEstimate", est.estimatedPose.toPose2d());

                });
        
    }



    public void periodic() {

        if (!Robot.isSimulation() && !robot.isAutonomous() && (pathRecounter % 10 == 0) && !this.disabledVision) {
            Pose2d currentPose = driveSub.getPose();
            updateVisionLocalization(currentPose);
            // var photonEstimate = getEstimatedGlobalPose(currentPose, "elevatorCam");

            // if (photonEstimate.isPresent()) {
            //     driveSub.addVisionMeasurement(photonEstimate.get().estimatedPose.toPose2d(), 
            //         photonEstimate.get().timestampSeconds
            //     );
            //     Logger.recordOutput("Drive/PhotonPoseEstimate", photonEstimate.get().estimatedPose.toPose2d());
            // }
        }

        advantageKitLogging();
    }

    ///////////////////////////////////
    // Pose stuff
    ///////////////////////////////////

    // public PhotonPipelineResult getLatestPoseResult() {

    //     return poseCamera.getLatestResult();
    // }

    // public Optional<PhotonPipelineResult> getLatestTrackingResult() {
    //     if (elevatorCamera.isConnected()) {
    //         return Optional.of(elevatorCamera.getLatestResult());
    //     } else {
    //         return Optional.empty();
    //     }
    // }

    ///////////////////////////////////
    // Tracking stuff
    ///////////////////////////////////

    // return the first target in the list with an ID we are interested in given our
    // alliance color
    // This uses the simple approach and assumes that we won't be seeing multiple
    // that we're interested in or that
    // the first one in the list is the best one.
    // public Optional<PhotonTrackedTarget> getBestTrackableTarget() {
    //     if (DriverStation.getAlliance().isPresent()) {
    //         for (var tempTarget : allDetectedTargets) {
    //             if (DriverStation.getAlliance().get() == Alliance.Red
    //                     && redTrackableIDs.contains(tempTarget.getFiducialId())) {
    //                 return Optional.of(tempTarget);
    //             }
    //             if (DriverStation.getAlliance().get() == Alliance.Blue
    //                     && blueTrackableIDs.contains(tempTarget.getFiducialId())) {
    //                 return Optional.of(tempTarget);
    //             }
    //         }
    //     }
    //     return Optional.empty();
    // }

    public Optional<PhotonTrackedTarget> getTarget(int id) {
        for (var tempTarget : allDetectedTargets) {
            if ((tempTarget.getFiducialId() == id)) {
                return Optional.of(tempTarget);
            }
        }
        return Optional.empty();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d robotPose) {
    //     if (elevatorCamera.isConnected()) {
    //         elevatorPhotonEstimator.setReferencePose(robotPose);
    //         var visionEst = elevatorPhotonEstimator.update(elevatorCamera.getLatestResult());
    //         double latestTimestamp = elevatorCamera.getLatestResult().getTimestampSeconds();
    //         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    //         if (Robot.isSimulation()) {
    //             visionEst.ifPresentOrElse(
    //                     est -> getSimDebugField()
    //                             .getObject("VisionEstimation")
    //                             .setPose(est.estimatedPose.toPose2d()),
    //                     () -> {
    //                         if (newResult)
    //                             getSimDebugField().getObject("VisionEstimation").setPoses();
    //                     });
    //         }
    //         if (newResult)
    //             lastEstTimestamp = latestTimestamp;
    //         return visionEst;
    //     } else {
    //         return Optional.empty();
    //     }
    // }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d robotPose, String cameraName) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if(elevatorCamera.getName().equals(cameraName) && elevatorCamera.isConnected()){
            ElevatorPhotonEstimator.setReferencePose(robotPose);
            for (var change : elevatorCamera.getAllUnreadResults()) {
                visionEst = ElevatorPhotonEstimator.update(change);
                //updateEstimationStdDevs(visionEst, m_cameraOneEstimator, change.getTargets());
            }
        } else if(climbCamera.getName().equals(cameraName) && rightClimbCam.isConnected()){
            rightClimbPhotonEstimator.setReferencePose(robotPose);
            for (var change : climbCamera.getAllUnreadResults()) {
                visionEst = rightClimbPhotonEstimator.update(change);
                // updateEstimationStdDevs(visionEst, m_cameraOneEstimator, change.getTargets());
            }
        }
        return visionEst;
    }


    // // wpk not used anywhere
    // public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    // var estStdDevs = SingleTagStdDevs;
    // var targets = getLatestPoseResult().getTargets();
    // int numTags = 0;
    // double avgDist = 0;
    // for (var tgt : targets) {
    // var tagPose = elevatorPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
    // if (tagPose.isEmpty())
    // continue;
    // numTags++;
    // avgDist +=
    // tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    // }
    // if (numTags == 0)
    // return estStdDevs;
    // avgDist /= numTags;
    // // Decrease stnd devs if multiple targets are visible
    // if (numTags > 1)
    // estStdDevs = MultiTagStdDevs;
    // // Increase stnd devs based on (average) distance
    // if (numTags == 1 && avgDist > 4)
    // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
    // Double.MAX_VALUE);
    // else
    // estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    // return estStdDevs;
    // }

    // ----- Simulation
    @Override
    public void simulationPeriodic() {
        visionSim.update(driveSub.getPose());
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation())
            visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }

    // public void chooseBestTarget() {
    // activeAlignTargetId = OptionalInt.empty();
    // if (allDetectedTargets != null) {
    // for (PhotonTrackedTarget target : allDetectedTargets) {
    // if (targetAlignSet.contains(target.getFiducialId())) {
    // activeAlignTargetId = OptionalInt.of(target.getFiducialId());
    // return;
    // }
    // }
    // }
    // }

    // public OptionalDouble getTargetRotOffSet() {
    // if (currentBestAlignTarget != null) {
    // return OptionalDouble.of(currentBestAlignTarget.getYaw());
    // }
    // return OptionalDouble.empty();
    // }

    // public OptionalDouble getTargetLateralOffSet() {
    // if (currentBestAlignTarget != null) {
    // // The pose of the april tag in the camera reference frame
    // var targetPose = currentBestAlignTarget.getBestCameraToTarget();

    // // targetYaw is the offset angle from camera-forward to the target
    // var targetYaw = currentBestAlignTarget.getYaw();

    // // Now find the "pointing angle of the target" in the reference frame
    // // of the robot. If you drew a ray from the target in space this would be
    // // the "center line" the robot wants to reach laterally.

    // // Note: this rotation value is a bit counter-intuitive. A Z-axis rotation of
    // 0
    // // actually means the April tag is pointed in the same direction as the
    // camera
    // var targetPointing = targetPose.getRotation().getZ();
    // // We want an angle about 0 when the tag is facing us so shift by Pi
    // // and then unroll.
    // targetPointing += Math.PI;
    // if (targetPointing > Math.PI) {
    // targetPointing -= 2 * Math.PI;
    // } else if (targetPointing < -Math.PI) {
    // targetPointing += 2 * Math.PI;
    // }
    // // Now combine the 2 angles to get the total angle between the robot and
    // // this center-line.
    // var combinedAngle = Math.toRadians(targetYaw) - targetPointing;

    // var targetDist = targetPose.getTranslation().getNorm();

    // // Finally, the lateral distance can be found by trig:
    // // Sin(angle) = Lateral / Hypotenuse
    // // Lateral = Hypotenuse * Sin(combined angle)
    // var lateralDist = Math.sin(combinedAngle) * targetDist;

    // return OptionalDouble.of(lateralDist);
    // }
    // return OptionalDouble.empty();
    // }

    // public OptionalDouble getTargetDistance() {
    // if (currentBestAlignTarget != null) {
    // var targetPose = currentBestAlignTarget.getBestCameraToTarget();
    // var targetDist = targetPose.getTranslation().getNorm();

    // return OptionalDouble.of(targetDist);
    // }
    // return OptionalDouble.empty();
    // }

    private void advantageKitLogging() {
        if (robotToCamera != null) {
            Logger.recordOutput("vision/xPosition", robotToCamera.getX());
            Logger.recordOutput("vision/yPosition", robotToCamera.getY());
            Logger.recordOutput("vision/zPosition", robotToCamera.getZ());
        }

        if (currentBestTarget != null) {
            Logger.recordOutput("vision/currentBestFiducial", currentBestTarget.getFiducialId());
            Logger.recordOutput("vision/bestCameraToTarget", currentBestTarget.getBestCameraToTarget());
        }
        // if (currentBestAlignTarget != null) {
        // Logger.recordOutput("vision/target/RotOffset",
        // getTargetRotOffSet().getAsDouble());
        // Logger.recordOutput("vision/target/LateralOffset",
        // getTargetLateralOffSet().getAsDouble());
        // Logger.recordOutput("vision/target/Distance",
        // getTargetDistance().getAsDouble());
        // }

        Logger.recordOutput("vision/targetAlignSet", targetAlignSet.toString());
        Logger.recordOutput("vision/activeAlignTargetStr", activeAlignTargetId.toString());
        if (activeAlignTargetId.isPresent()) {
            Logger.recordOutput("vision/activeAlignTarget", activeAlignTargetId.getAsInt());
        }
    }

    public boolean coralIsVisible() {
        return this.coralDetected;
    }
    public double getCoralDistanceFromCenter() {
        // tell how many pixels the note is from the center of the screen.
        return this.coralDistanceFromCenter;
    }

    public double getCoralHeight() {
        return this.coralHeight;
    }

    public double getCoralWidth() {
        return this.coralWidth;
    }
  
    public List<PhotonTrackedTarget> getAllDetectedTargets() {
        return this.allDetectedTargets;
    }
}
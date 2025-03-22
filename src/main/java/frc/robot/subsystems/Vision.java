
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.VisionConstants.ClimbCameraName;
import static frc.robot.Constants.VisionConstants.ClimbCameraToRobot;
import static frc.robot.Constants.VisionConstants.ElevatorCamToRobot;
import static frc.robot.Constants.VisionConstants.ElevatorCameraName;
import static frc.robot.Constants.VisionConstants.FallbackVisionStrategy;
import static frc.robot.Constants.VisionConstants.FieldTagLayout;
import static frc.robot.Constants.VisionConstants.PrimaryVisionStrategy;
import static frc.robot.Constants.VisionConstants.RightClimbCamName;
import static frc.robot.Constants.VisionConstants.RightClimbCamToRobot;
// import static frc.robot.Constants.VisionConstants.RobotToElevatorCam;
// import static frc.robot.Constants.VisionConstants.RobotToRightClimbCam;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    ////// ------ PHOTON VISION AND APRIL TAG VARIABLES ------ //////

    private PhotonCamera elevatorCamera;
    private PhotonCamera climbCamera;
    private PhotonCamera rightClimbCam;
    public final PhotonPoseEstimator elevatorPhotonEstimator;
    public final PhotonPoseEstimator rightClimbPhotonEstimator;
    private boolean disabledVision = false;
    private PhotonCameraSim climbCameraSim;
    private PhotonCameraSim elevatorCameraSim;
    private VisionSystemSim visionSim;
    private Transform3d robotToCamera;
    public List<PhotonTrackedTarget> allDetectedTargets;
    boolean aprilTagDetected = false;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final Drive driveSub;
    
    // shouldn't need robot in here WPK
    private final Robot robot;

    // wpk - how do we figure out the correct values to use below?

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private Matrix<N3, N1> curStdDevs;




    public Vision(Drive driveSub, Robot robot) /* throws IOException */ {
        this.driveSub = driveSub;
        this.robot = robot;
        elevatorCamera = new PhotonCamera(ElevatorCameraName); // left camera
        rightClimbCam = new PhotonCamera(RightClimbCamName);
        climbCamera = new PhotonCamera(ClimbCameraName); // right camera

        // elevatorPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, poseCamera, PoseCameraToRobot);
        elevatorPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, ElevatorCamToRobot);
        elevatorPhotonEstimator.setMultiTagFallbackStrategy(FallbackVisionStrategy);

        rightClimbPhotonEstimator = new PhotonPoseEstimator(FieldTagLayout, PrimaryVisionStrategy, RightClimbCamToRobot);
        rightClimbPhotonEstimator.setMultiTagFallbackStrategy(FallbackVisionStrategy);

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
            visionSim.addCamera(elevatorCameraSim, ElevatorCamToRobot);

            climbCameraSim.enableDrawWireframe(true);
        }

    }

    public AprilTagFieldLayout getLayout() {
        return this.aprilTagFieldLayout;
    }


    public void disableTheVision(boolean val) {
        this.disabledVision = val;
    }

    public void updateVisionLocalization(Pose2d drivePose) {
        var visionEst = getEstimatedGlobalPose(drivePose, elevatorCamera, elevatorPhotonEstimator);
        visionEst.ifPresent(
                est -> {
                    // driveSub.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    addVisionMeasure(est);
                    Logger.recordOutput("Vision/ElevatorCameraPoseEstimate", est.estimatedPose.toPose2d());

                    // m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraOneStdDev);
                    // m_PoseEstimator.addVisionMeasurement(
                    //         est.estimatedPose.toPose2d(), est.timestampSeconds);
                });

        visionEst = getEstimatedGlobalPose(drivePose, rightClimbCam, rightClimbPhotonEstimator);
        visionEst.ifPresent(
                est -> {
                    addVisionMeasure(est);
                    // m_PoseEstimator.setVisionMeasurementStdDevs(Constants.vision.localizationCameraTwoStdDev);
                    // driveSub.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
                    Logger.recordOutput("Vision/RightClimbPoseEstimate", est.estimatedPose.toPose2d());

                });
        
    }


    @Override
    public void periodic() {

        if (!Robot.isSimulation() && !robot.isAutonomous() && /*(!robot.currentlyFollowingAPath || pathRecounter % 10 == 0) &&*/ !this.disabledVision) {
            Pose2d currentPose = driveSub.getPose();
            updateVisionLocalization(currentPose);
        }

        advantageKitLogging();
    }

    ///////////////////////////////////
    // Pose stuff
    ///////////////////////////////////


    // wpk this is different than the example in the photon documentation. Should it be?

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d robotPose, PhotonCamera camera, PhotonPoseEstimator poseEstimator) {

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        // wpk do we need to check if the camera is connected? Will the pose estimator do that for us automatically?
        if(camera.isConnected()){
            poseEstimator.setReferencePose(robotPose);
            for (var change : camera.getAllUnreadResults()) {
                visionEst = poseEstimator.update(change);
                //updateEstimationStdDevs(visionEst, change.getTargets(), poseEstimator);
            }
        }
        return visionEst ;
    }





/**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }        





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


    private void advantageKitLogging() {

        var closestPose = findPoseOfTagClosestToRobot(driveSub.getPose()) ;
        if ( closestPose.isPresent()) {
            Logger.recordOutput("vision/ClosestAprilTag", closestPose.get() );
        }
        if (robotToCamera != null) {
            Logger.recordOutput("vision/RobotToCamera", robotToCamera );
        }
    }


  
    public List<PhotonTrackedTarget> getAllDetectedTargets() {
        return this.allDetectedTargets;
    }


    public Matrix<N3, N1> addVisionMeasure(EstimatedRobotPose estimated_pose) {
        Pose2d pose = estimated_pose.estimatedPose.toPose2d();
        double visionTime = estimated_pose.timestampSeconds;
        var tags = estimated_pose.targetsUsed;
        int tagCount = tags.size();
        
        if (tagCount == 0) {
            return null;
        }
    
        ArrayList<Integer> tagIds = new ArrayList<>();
        for (PhotonTrackedTarget tag : tags) {
            tagIds.add(tag.fiducialId);
        }
    
        int primaryId = tagIds.get(0);
        double distanceToTarget = tags.get(0).bestCameraToTarget.getTranslation().toTranslation2d().getDistance(new Translation2d(0, 0));
        
        double stdDev = 2;
        Logger.recordOutput("Vision/DistanceToTarget", distanceToTarget);
    
        if (tagCount == 1) {
            if (distanceToTarget > 2.5) {
                return null;
            }
            if (((primaryId >= 6 && primaryId <= 11) || (primaryId >= 17 && primaryId <= 22)) && distanceToTarget <= 1.5) {
                stdDev = 0.25;
                if (distanceToTarget <= 0.75) {
                    stdDev = 0.1;
                    if (DriverStation.isTeleop()) {
                        driveSub.addVisionMeasurement(pose, visionTime, VecBuilder.fill(stdDev, stdDev, stdDev));
                        return null;
                    }
                }
            }
        } else if (tagCount >= 2) {
            stdDev = 0.7;
            if (((primaryId >= 6 && primaryId <= 11) || (primaryId >= 17 && primaryId <= 22)) && distanceToTarget <= 0.5) {
                stdDev = 0.5;
                if (distanceToTarget <= 0.25) {
                    stdDev = 0.25;
                }
            }
        }
        Logger.recordOutput("Vision/stdDev", distanceToTarget);



        // size of the angles. 

        driveSub.addVisionMeasurement(
            new Pose2d(pose.getX(), pose.getY(), driveSub.getPose().getRotation()), 
            visionTime, 
            VecBuilder.fill(stdDev, stdDev, 50)
        );
    
        return null;
    }



    Optional<Pose2d> findPoseOfTagClosestToRobot(Pose2d drivePose) {

        int[] aprilTagList;
        var alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            return Optional.empty();
        }

        if (alliance.get() == DriverStation.Alliance.Blue) {
            aprilTagList = Constants.VisionConstants.blueAprilTagListReef;
        } else if (alliance.get() == DriverStation.Alliance.Red) {
            aprilTagList = Constants.VisionConstants.redAprilTagListReef;
        } else {
            // Short circuit so we don't end up with an empty list
            return Optional.empty();
        }

        List<Pose2d> possiblePoses = new ArrayList<>();

        for (int id : aprilTagList) {
            possiblePoses.add(getLayout().getTagPose(id).get().toPose2d());
        }

        return Optional.of(drivePose.nearest(possiblePoses));
    }

}
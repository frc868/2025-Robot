package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.techhounds.houndutil.houndlog.FaultLogger;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;
import com.techhounds.houndutil.houndlog.annotations.Log;

public class ObjectPhotonCamera {
    /**
     * Common constants for a camera intended to view Objects
     */
    public static class ObjectCameraConstants {
        /** The width of the image, in pixels. */
        public int WIDTH;
        /** The height of the image, in pixels. */
        public int HEIGHT;
        /** The diagonal field of view of the camera, in degrees. */
        public double FOV;
        /** The expected FPS of the camera. Used for simulation. */
        public double FPS;
        /**
         * The expected average latency, in ms, of the camera. Used for simulation. If
         * unsure, use 30ms.
         */
        public double AVG_LATENCY;
        /**
         * The expected standard deviation of the latency, in ms, of the camera. Used
         * for simulation. If unsure, use 15ms.
         */
        public double STDDEV_LATENCY;
    }

    /**
     * Yaw, pitch, confidence, ID, and area of detected object in one convenient
     * class
     */
    public class ObjectBearing {
        public double yaw;
        public double pitch;
        public float conf;
        public int id;
        public double area;
    }

    private String name;
    private PhotonCamera photonCamera;
    private PhotonCameraSim cameraSim;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d robotToCam;

    @Log
    private Pose3d[] detectedObjects = new Pose3d[0];
    @Log
    private boolean hasPose = false;
    @Log
    private int targetCount = 0;

    private double lastTimestamp = 0;

    /**
     * Initializes the PhotonVision camera.
     * 
     * @param name          the name of the camera assigned in PhotonVision.
     * @param robotToCam    the transform from the center of the robot (at a
     *                      z-height of 0) to the sensor of the camera. this should
     *                      be as accurate as possible to minimize compounding
     *                      tolerances.
     * @param constants     the common constants for the camera.
     * @param avgErrorPx    the average error of the camera calibration, used for
     *                      simulation. if unsure, use 0.2px.
     * @param stdDevErrorPx the standard deviation of the error of the camera
     *                      calibration, used for simulation. if unsure, use 0.1px.
     */
    public ObjectPhotonCamera(String name, Transform3d robotToCam, ObjectCameraConstants constants,
            double avgErrorPx, double stdDevErrorPx) {
        this.name = name;
        this.robotToCam = robotToCam;

        photonCamera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (RobotBase.isSimulation()) {
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(constants.WIDTH, constants.HEIGHT,
                    Rotation2d.fromDegrees(constants.FOV));
            cameraProp.setCalibError(avgErrorPx, stdDevErrorPx);
            cameraProp.setFPS(constants.FPS);
            cameraProp.setAvgLatencyMs(constants.AVG_LATENCY);
            cameraProp.setLatencyStdDevMs(constants.STDDEV_LATENCY);
            cameraSim = new PhotonCameraSim(photonCamera, cameraProp);

            cameraSim.enableDrawWireframe(true);
        }
        FaultLogger.register(photonCamera);
    }

    /**
     * Refreshes cam and Gets the robot relative pose3d locations for all detected
     * targets
     * 
     * @return list of robot relative Transform3d locations for all detected objects
     */
    public Optional<ObjectBearing[]> getDetectedObjects() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(timestamp - lastTimestamp) > 1e-5;
        targetCount = result.targets.size();

        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();

        if (newResult) {
            List<PhotonTrackedTarget> targets = result.targets;
            List<ObjectBearing> bearings = new ArrayList<ObjectBearing>();

            for (int i = 0; i < targetCount; i++) {
                ObjectBearing currentObject = new ObjectBearing();
                currentObject.yaw = targets.get(i).getYaw();
                currentObject.pitch = targets.get(i).getPitch();
                currentObject.conf = targets.get(i).getDetectedObjectConfidence();
                currentObject.id = targets.get(i).getDetectedObjectClassID();
                currentObject.area = targets.get(i).getArea();
            }

            ObjectBearing bearingArray[] = new ObjectBearing[bearings.size()];
            bearings.toArray(bearingArray);
            lastTimestamp = timestamp;
            return Optional.of(bearingArray);
        }
        return Optional.empty();

    }

    /**
     * Gets the name of the camera.
     * 
     * @return the name of the camera
     */
    public String getName() {
        return name;
    }

    /**
     * Gets whether the camera is currently producing a pose measurement.
     * 
     * @return if the camera is currently producing a pose measurement
     */
    public boolean hasPose() {
        return hasPose;
    }

    /**
     * Gets the simulation object of the camera, to add to a global vision
     * simulator.
     * 
     * @return the underlying {@link PhotonCameraSim} representing this camera
     */
    public PhotonCameraSim getSim() {
        return cameraSim;
    }

    /**
     * Gets the provided transformation from the center of the robot to the sensor
     * of the camera.
     * 
     * @return the transformation from the center of the robot to the sensor of the
     *         camera
     */
    public Transform3d getRobotToCam() {
        return robotToCam;
    }
}

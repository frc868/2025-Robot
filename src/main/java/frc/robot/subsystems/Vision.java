package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.TriConsumer;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;
import com.techhounds.houndutil.houndlog.annotations.Log;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.subsystems.Vision.Constants.*;
import frc.robot.subsystems.ObjectPhotonCamera;
import frc.robot.subsystems.ObjectPhotonCamera.*;
import frc.utils.TargetAlign;

public class Vision implements BaseVision {
    public static final class Constants {
        public static final float MIN_OBJECT_CONFIDENCE = 0.8f;
        public static final int ALGAE_OBJECT_ID = 1; // TODO

        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE); // TODO
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1,
                Double.MAX_VALUE); // TODO
        public static final Matrix<N3, N1> MULTI_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.01, 0.01,
                Double.MAX_VALUE); // TODO

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600; // TODO
            CAMERA_CONSTANTS.HEIGHT = 1200; // TODO
            CAMERA_CONSTANTS.FOV = 95.39; // TODO
            CAMERA_CONSTANTS.FPS = 35; // TODO
            CAMERA_CONSTANTS.AVG_LATENCY = 30; // TODO
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15; // TODO
        }

        public static final ObjectCameraConstants OBJECT_CAMERA_CONSTANTS = new ObjectCameraConstants();
        static {
            OBJECT_CAMERA_CONSTANTS.WIDTH = 1600; // TODO
            OBJECT_CAMERA_CONSTANTS.HEIGHT = 1200; // TODO
            OBJECT_CAMERA_CONSTANTS.FOV = 95.39; // TODO
            OBJECT_CAMERA_CONSTANTS.FPS = 35; // TODO
            OBJECT_CAMERA_CONSTANTS.AVG_LATENCY = 30; // TODO
            OBJECT_CAMERA_CONSTANTS.STDDEV_LATENCY = 15; // TODO

        }

        public static final int[] sideToTagIDBlue = { 18, 17, 22, 21, 20, 19 };
        public static final int[] sideToTagIDRed = { 7, 8, 9, 10, 11, 6 };

        // 2/17/24
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                // front camera
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(11.886316),
                                -Units.inchesToMeters(7.507594),
                                Units.inchesToMeters(9.541569)), // TODO
                        new Rotation3d(0, Units.degreesToRadians(-25),
                                Units.degreesToRadians(10))), // TODO
                // left camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)), // TODO
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(70))), // TODO
                // right camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                -Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)), // TODO
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(-70))), // TODO
                // object camera
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0)),
                        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)))
        };
    }

    /**
     * The pose estimator to receive the latest robot position from (used for
     * logging and the "closest to last pose" strategy).
     */
    private SwerveDrivePoseEstimator poseEstimator = null;
    /**
     * The consumer for vision measurements, taking in the pose, the timestamp, and
     * the standard deviations.
     */
    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer = null;
    /**
     * Supplier for drivetrain chassis speeds, used to disable pose estimation when
     * moving too quickly.
     */
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = null;
    /**
     * Pose supplier for a ground source of truth pose via odometry, for simulation.
     */
    private Supplier<Pose2d> simPoseSupplier = null;
    /** A simulation of the vision system. */
    private final VisionSystemSim visionSim = new VisionSystemSim("main");

    @Log(groups = "cameras")
    private final AprilTagPhotonCamera frontLeftCam = new AprilTagPhotonCamera("FrontLeft",
            ROBOT_TO_CAMS[0], CAMERA_CONSTANTS, 0.2, 0.1);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera frontRightCam = new AprilTagPhotonCamera("FrontRight",
            ROBOT_TO_CAMS[1], CAMERA_CONSTANTS, 0.2, 0.1);
    @Log(groups = "cameras")
    private final AprilTagPhotonCamera backCam = new AprilTagPhotonCamera("Back",
            ROBOT_TO_CAMS[2], CAMERA_CONSTANTS, 0.2, 0.1);
    @Log(groups = "cameras")
    private final ObjectPhotonCamera objectCam = new ObjectPhotonCamera("Object",
            ROBOT_TO_CAMS[3], OBJECT_CAMERA_CONSTANTS, 0.2, 0.1);

    private final AprilTagPhotonCamera[] cameras = new AprilTagPhotonCamera[] {
            frontLeftCam, frontRightCam, backCam };

    public Vision() {
        AprilTagFieldLayout tagLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(tagLayout);
            for (AprilTagPhotonCamera camera : cameras) {
                visionSim.addCamera(camera.getSim(), camera.getRobotToCam());
            }
        }
    }

    @Override
    public void updatePoseEstimator() {
        if (poseEstimator == null) {
            return;
        }

        Pose2d prevEstimatedRobotPose = poseEstimator.getEstimatedPosition();
        for (AprilTagPhotonCamera photonCamera : cameras) {
            Optional<EstimatedRobotPose> result = photonCamera
                    .getEstimatedGlobalPose(prevEstimatedRobotPose);

            if (result.isPresent()) {
                EstimatedRobotPose estPose = result.get();
                Pose2d pose = estPose.estimatedPose.toPose2d();

                Matrix<N3, N1> stddevs = photonCamera.getEstimationStdDevs(pose,
                        SINGLE_TAG_STD_DEVS,
                        DriverStation.isAutonomous() ? MULTI_TAG_STD_DEVS
                                : MULTI_TAG_TELEOP_STD_DEVS);

                double normSpeed = new Translation2d(chassisSpeedsSupplier.get().vxMetersPerSecond,
                        chassisSpeedsSupplier.get().vyMetersPerSecond).getNorm();
                if (normSpeed < 0.5 || !DriverStation.isAutonomous()) {
                    if (photonCamera.getName() == "HoundEye01" || !DriverStation.isAutonomous()) {
                        visionMeasurementConsumer.accept(pose, Timer.getFPGATimestamp(),
                                stddevs);
                    }
                }
            }
        }
    }

    @Override
    public Pose3d[] getCameraPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (Transform3d transform : ROBOT_TO_CAMS) {
            poses.add(new Pose3d(poseEstimator.getEstimatedPosition()).plus(transform)
                    .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI))));
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    @Override
    public Pose3d[] getAprilTagPoses() {
        List<Pose3d> poses = new ArrayList<Pose3d>();
        for (AprilTag tag : AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTags()) {
            poses.add(tag.pose);
        }
        Pose3d[] poseArray = new Pose3d[poses.size()];
        return poses.toArray(poseArray);
    }

    @Override
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        this.simPoseSupplier = simPoseSupplier;
    }

    /**
     * Sets the consumer for vision measurements, taking in the pose, the timestamp,
     * and the standard deviations of a given measurement.
     * 
     * @param visionMeasurementConsumer the consumer to use
     */
    public void setVisionMeasurementConsumer(
            TriConsumer<Pose2d, Double, Matrix<N3, N1>> visionMeasurementConsumer) {
        this.visionMeasurementConsumer = visionMeasurementConsumer;
    }

    /**
     * Sets the supplier for the robot's chassis speeds, used to invalidate pose
     * measurements when the robot is moving too fast.
     * 
     * @param chassisSpeedsSupplier the supplier to use
     */
    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    /**
     * Gets an aggregated list of the latest cached measurements from all cameras,
     * so that they can be displayed easily. Does not actually update the cameras.
     * 
     * @return the latest cached measurements from all cameras
     */
    @Log
    public Pose3d[] getEstimatedRobotPoses() {
        return new Pose3d[] {
                frontLeftCam.getLoggedEstimatedRobotPose(),
                frontRightCam.getLoggedEstimatedRobotPose(),
                backCam.getLoggedEstimatedRobotPose() };
    }

    /**
     * Gets an aggregated list of all of the detected AprilTags from the latest
     * cached measurements from all cameras, so that they can be displayed easily.
     * Does not actually update the cameras.
     * 
     * @return the latest cached detected AprilTags from all cameras
     */
    @Log
    public Pose3d[] getDetectedAprilTags() {
        Pose3d[][] detectedTags = {
                frontLeftCam.getLoggedDetectedAprilTags(),
                frontRightCam.getLoggedDetectedAprilTags(),
                backCam.getLoggedDetectedAprilTags()
        };

        int totalSize = 0;
        for (Pose3d[] tags : detectedTags) {
            totalSize += tags.length;
        }

        Pose3d[] result = new Pose3d[totalSize];

        int index = 0;
        for (Pose3d[] tags : detectedTags) {
            for (Pose3d tag : tags) {
                result[index++] = tag;
            }
        }

        return result;
    }

    /**
     * Gets the yaw of the closest
     * 
     * @return the position of the closest algae piece
     */
    @Log
    public double getClosestAlgaeYaw() {
        Optional<ObjectBearing[]> optionalBearings = objectCam.getDetectedObjects();
        ObjectBearing[] bearings = optionalBearings.orElse(new ObjectBearing[0]);
        double largestArea = 0;
        double largestYaw = -10000.0;

        if (bearings.length == 0) {
            return -10000.0;
        }
        for (int i = 0; i < bearings.length; i++) {
            if (bearings[i].id == ALGAE_OBJECT_ID) {
                if (bearings[i].conf > MIN_OBJECT_CONFIDENCE) {
                    if (bearings[i].area > largestArea) {
                        largestYaw = Units.degreesToRadians(bearings[i].yaw);
                        largestArea = bearings[i].area;
                    }
                }
            }
        }

        if (largestArea == 0)
            return -10000.0;
        return largestYaw;
    }

    /**
     * Provides the pose3d needed to be driven to to score based on the front 2
     * cameras, assuming both can see the tag related to the scoring position.
     * Returns null if cannot detect tag.
     * 
     * @param tagID the ID of the tag to score on
     * @param left  whether to score on the leftside or rightside.
     */
    @Log
    public Pose2d getScoringPosition(int side, boolean left) {
        int tagID;
        if (DriverStation.getAlliance().orElse(null) == Alliance.Red) {
            tagID = sideToTagIDRed[side];
        } else {
            tagID = sideToTagIDBlue[side];
        }

        Transform3d transforms[] = new Transform3d[2];
        Transform3d averagedTransform = new Transform3d();
        int numCanSeeTag = 0;
        for (int i = 0; i < 2; i++) {
            Transform3d transform = cameras[i].getRelativeTransformsOfTarget(tagID);
            if (transform == null) {
                continue;
            } else {
                transforms[i] = transform;
                numCanSeeTag++;
            }
        }

        if (numCanSeeTag++ < 1) {
            return null;
        } else {
            for (int i = 0; i < 2; i++) {
                averagedTransform = averagedTransform.plus(transforms[i]);
            }
            averagedTransform = averagedTransform.times(1.0 / numCanSeeTag);
        }
        Transform2d transform = new Transform2d(
                new Translation2d(averagedTransform.getX(), averagedTransform.getY()),
                new Rotation2d(averagedTransform.getRotation().getZ()));

        updatePoseEstimator();
        Pose2d targetPose = new Pose2d(
                poseEstimator.getEstimatedPosition().plus(transform)
                        .plus(TargetAlign.getTargetTransformFromAprilTag(left, side)).getX(),
                poseEstimator.getEstimatedPosition().plus(transform)
                        .plus(TargetAlign.getTargetTransformFromAprilTag(left, side)).getY(),
                TargetAlign.getTargetRotation(side));
        return targetPose;
    }
}
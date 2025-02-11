package frc.robot.subsystems;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.subsystems.BaseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Vision implements BaseVision {
        public static final class Constants {
                public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(Double.MAX_VALUE,
                                Double.MAX_VALUE,
                                Double.MAX_VALUE); // TODO
                public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE); // TODO
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
                                                                Units.degreesToRadians(-70))) // TODO
                };
        }

        @Override
        public void updatePoseEstimator() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstimator'");
        }

        @Override
        public Pose3d[] getCameraPoses() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getCameraPoses'");
        }

        @Override
        public Pose3d[] getAprilTagPoses() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getAprilTagPoses'");
        }

        @Override
        public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'setPoseEstimator'");
        }

        @Override
        public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'setSimPoseSupplier'");
        }
}
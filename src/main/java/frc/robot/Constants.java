package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
    enum ControllerType {
        XboxController,
        FlightStick
    }

    public static final boolean DEBUG_MODE = false;

    public static final ControllerType CONTROLLER_TYPE = ControllerType.FlightStick;

    public static final double PERIOD = 0.020;

    
    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(Double.MAX_VALUE,
                Double.MAX_VALUE,
                Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);

        public static final PhotonCameraConstants CAMERA_CONSTANTS = new PhotonCameraConstants();
        static {
            CAMERA_CONSTANTS.WIDTH = 1600;
            CAMERA_CONSTANTS.HEIGHT = 1200;
            CAMERA_CONSTANTS.FOV = 95.39;
            CAMERA_CONSTANTS.FPS = 35;
            CAMERA_CONSTANTS.AVG_LATENCY = 30;
            CAMERA_CONSTANTS.STDDEV_LATENCY = 15;
        }

        // 2/17/24
        public static final Transform3d[] ROBOT_TO_CAMS = new Transform3d[] {
                // front camera
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(11.886316),
                                -Units.inchesToMeters(7.507594),
                                Units.inchesToMeters(9.541569)),
                        new Rotation3d(0, Units.degreesToRadians(-25),
                                Units.degreesToRadians(10))),
                // left camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(70))),
                // right camera
                new Transform3d(
                        new Translation3d(
                                -Units.inchesToMeters(1.765373),
                                -Units.inchesToMeters(10.707761),
                                Units.inchesToMeters(12.116848)),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(-70)))
        };
    }

    public static final class Teleop {
        /**
         * A value inputted into the rate limiter (the joystick input) can move from 0
         * to 1 in 1/RATE_LIMIT seconds.
         * 
         * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
         * Larger numbers mean less of a rate limit.
         */
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0; // TODO
        public static final double JOYSTICK_INPUT_DEADBAND = 0.05; // TODO
        public static final double JOYSTICK_CURVE_EXP = 2; // TODO
        public static final double JOYSTICK_ROT_CURVE_EXP = 1; // TODO
    }

    public static final class HoundBrian {
        public static final int BUTTON_1 = 3;
        public static final int BUTTON_2 = 4;
        public static final int BUTTON_3 = 5;
        public static final int BUTTON_4 = 6;
        public static final int BUTTON_5 = 7;
        public static final int BUTTON_6 = 8;
        public static final int BUTTON_7 = 9;
    }
}
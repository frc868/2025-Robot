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

    public static final class Drivetrain {

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 4;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 8;

        public static final int FRONT_LEFT_STEER_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_STEER_ENCODER_ID = 1;
        public static final int BACK_LEFT_STEER_ENCODER_ID = 2;
        public static final int BACK_RIGHT_STEER_ENCODER_ID = 3;

        public static final String CAN_BUS_NAME = "canivore";

        public static final int PIGEON_ID = 0;

        public static final boolean DRIVE_MOTORS_INVERTED = false;
        public static final boolean STEER_MOTORS_INVERTED = true;
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : true;

        // 2/17/24
        // public static final double FRONT_LEFT_OFFSET = 0.457763671875;
        // public static final double FRONT_RIGHT_OFFSET = -0.183349609375;
        // public static final double BACK_LEFT_OFFSET = 0.24267578125;
        // public static final double BACK_RIGHT_OFFSET = 0.48583984375;
        public static final double FRONT_LEFT_OFFSET = 0.4521484375;
        public static final double FRONT_RIGHT_OFFSET = -0.179443359375 - 0.00634765625;
        public static final double BACK_LEFT_OFFSET = 0.242919921875;
        public static final double BACK_RIGHT_OFFSET = 0.498046875 - 0.003;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.527;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.527;
        public static final double DRIVE_BASE_RADIUS_METERS = 0.3727;


        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            // 2/24/24
            SWERVE_CONSTANTS.DRIVE_kP = 0.84992;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.2368;
            SWERVE_CONSTANTS.DRIVE_kV = 0.67229;
            SWERVE_CONSTANTS.DRIVE_kA = 0.080151;
            SWERVE_CONSTANTS.STEER_kP = 100.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0;
            SWERVE_CONSTANTS.STEER_kS = 0; // TODO
            SWERVE_CONSTANTS.STEER_kV = 0; // TODO
            SWERVE_CONSTANTS.STEER_kA = 0; // TODO

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.357;
            SWERVE_CONSTANTS.STEER_GEARING = 150.0 / 7.0;
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 16.0;
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.0491630791391;
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.54;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8; // TODO
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI; // TODO
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 100 * 2 * Math.PI; // TODO

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 100;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 30;

        public static final Translation2d[] SWERVE_MODULE_LOCATIONS = new Translation2d[] {
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2) };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                SWERVE_MODULE_LOCATIONS[0],
                SWERVE_MODULE_LOCATIONS[1],
                SWERVE_MODULE_LOCATIONS[2],
                SWERVE_MODULE_LOCATIONS[3]);

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0;
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0;

        public static final double XY_kP = 1.4;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0.05;
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 1.3;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.05;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double MASS = 50.0; // KG, TODO
        public static final double MOMENT_OF_INERTIA = 0.05; // KG m^2, TODO
        public static final double SWERVE_WHEEL_RADIUS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE / (Math.PI * 2.0); // meters
        
        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(SWERVE_WHEEL_RADIUS, SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND, 1.0, SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR, SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1);
        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS, 0.5, MODULE_CONFIG, DRIVE_BASE_RADIUS_METERS);
    }

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
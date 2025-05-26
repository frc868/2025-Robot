package frc.robot;

import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;
import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;
import com.techhounds.houndutil.houndlog.loggers.MetadataLogger.MetadataRecord;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.FieldConstants.Reef.ReefBranch;
import frc.robot.subsystems.LEDs.LEDState;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    public static final MetadataRecord BUILD_METADATA = new MetadataRecord(
            BuildConstants.MAVEN_NAME, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE,
            BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);

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

        public static final TunableDouble DEMO_SPEED = new TunableDouble("subsystems/drivetrain/DEMO_SPEED", 1.0);

        public static final boolean DRIVE_MOTORS_INVERTED = false;
        public static final boolean STEER_MOTORS_INVERTED = true;
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : true;

        public static final double FRONT_LEFT_OFFSET = 0.1108398438 + 0.005126953125;
        public static final double FRONT_RIGHT_OFFSET = 0.373046875 - 0.000732421875;
        public static final double BACK_LEFT_OFFSET = -0.0278320313 + 0.002197265625;
        public static final double BACK_RIGHT_OFFSET = -0.2575683594 - 0.003662109375;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.60325;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.60325;
        /** Distance between the center of the robot and the */
        public static final double DRIVE_BASE_RADIUS_METERS = 0.4265621658;
        public static final double MASS_KG = Units.lbsToKilograms(135);
        public static final double MOI = 6.0; // TODO sim value
        public static final double WHEEL_RADIUS_METERS = 0.0491660284;
        public static final double WHEEL_COF = 1.3; // TODO sim value, this is a good guess for TPU

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 1.3;
            SWERVE_CONSTANTS.DRIVE_kI = 0.0;
            SWERVE_CONSTANTS.DRIVE_kD = 0.0;
            SWERVE_CONSTANTS.DRIVE_kS = 0.14609;
            SWERVE_CONSTANTS.DRIVE_kV = 0.73522;
            SWERVE_CONSTANTS.DRIVE_kA = 0.096345;
            SWERVE_CONSTANTS.STEER_kP = 100.0;
            SWERVE_CONSTANTS.STEER_kI = 0.0;
            SWERVE_CONSTANTS.STEER_kD = 1.0;
            SWERVE_CONSTANTS.STEER_kS = 0;
            SWERVE_CONSTANTS.STEER_kV = 0;
            SWERVE_CONSTANTS.STEER_kA = 0;

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.9027777778;
            SWERVE_CONSTANTS.STEER_GEARING = 18.75;
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 16.0; // TODO not sure what the coupling ratio would be
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS_METERS; // TODO sim value
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            // TODO this is free speed, but should be measured ground speed
            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = RobotBase.isReal() ? 4.93 : 5.29;
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 5.41; // TODO sim value
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI;
            // max velocity in 1/10 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 100 * 2 * Math.PI;

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 85;
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30;
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01;
            SWERVE_CONSTANTS.STEER_MOI = 0.025;
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 11.47;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 30; // TODO

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

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.5;
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0;

        public static final double XY_kP = 8;
        public static final double XY_kI = 0;
        public static final double XY_kD = 0.05;

        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                4,
                4);
        public static final TrapezoidProfile.Constraints XY_TALL_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.5,
                2.5);
        public static final TrapezoidProfile.Constraints AUTO_FAST_CONSTRAINTS = new TrapezoidProfile.Constraints(
                4.0,
                4.5);
        public static final TrapezoidProfile.Constraints AUTO_TALL_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.5,
                2.5);
        public static final TrapezoidProfile.Constraints AUTO_STANDARD_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.2,
                1.7);

        // lifted from 6328's DriveToPose
        public static final double XY_FF_MIN_RANGE = 0.1;
        public static final double XY_FF_MAX_RANGE = 0.15;

        public static final double THETA_kP = 8;
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.1;
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                5,
                12);
        public static final TrapezoidProfile.Constraints THETA_SLOW_CONSTRAINTS = new TrapezoidProfile.Constraints(
                5,
                3);

        public static final double ALGAE_DRIVE_kP = 5;
        public static final double ALGAE_DRIVE_kI = 0;
        public static final double ALGAE_DRIVE_kD = 0;
        public static final TrapezoidProfile.Constraints ALGAE_DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(
                2.5,
                1.5);

        public static final double ALGAE_THETA_kP = 0.1;
        public static final double ALGAE_THETA_kI = 0;
        public static final double ALGAE_THETA_kD = 0.005;
        public static final TrapezoidProfile.Constraints ALGAE_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                5,
                12);

        public static final double ALGAE_AREA_TARGET = 42;

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                MASS_KG, MOI,
                new ModuleConfig(
                        WHEEL_RADIUS_METERS,
                        SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                        WHEEL_COF, SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                        SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1),
                SWERVE_MODULE_LOCATIONS);

        public static final double SCORING_DISTANCE_METERS = 0.6865896 + 0.05;
        public static final double ALGAE_DESCORE_DISTANCE = 0.65;
        public static final double TROUGH_OFFSET_METERS = 0.31;
        public static final Transform2d L2_L3_SCORING_TRANSFORM = new Transform2d(-0.03, 0, Rotation2d.kZero);

        public static final Pose2d NET_SCORE_LINE = new Pose2d(7.913, 0, Rotation2d.kCW_90deg);
        public static final Pose2d NET_SCORE_AUTO = new Pose2d(7.913, 5.0, Rotation2d.k180deg);
        public static final Rotation2d NET_SCORE_ROTATION_TRANSFORM = Rotation2d.kCW_90deg;

        // Practice Field
        // public static final Map<ReefBranch, Transform2d> scoringTransformsRed =
        // Map.ofEntries(
        // Map.entry(ReefBranch.A, new Transform2d(0, -.035, Rotation2d.kZero)),
        // Map.entry(ReefBranch.B, new Transform2d(0, -.02, Rotation2d.kZero)),
        // Map.entry(ReefBranch.C, new Transform2d(-0.05, 0.005, Rotation2d.kZero)),
        // Map.entry(ReefBranch.D, new Transform2d(-0.061, -0.015, Rotation2d.kZero)),
        // Map.entry(ReefBranch.E, new Transform2d(-0.025, 0.01, Rotation2d.kZero)),
        // Map.entry(ReefBranch.F, new Transform2d(-0.04, -0.029, Rotation2d.kZero)),
        // Map.entry(ReefBranch.G, new Transform2d(-0.004, -0.027, Rotation2d.kZero)),
        // Map.entry(ReefBranch.H, new Transform2d(-0.003, -0.013, Rotation2d.kZero)),
        // Map.entry(ReefBranch.I, new Transform2d(-0.005, 0.01, Rotation2d.kZero)),
        // Map.entry(ReefBranch.J, new Transform2d(-0.003, 0.001, Rotation2d.kZero)),
        // Map.entry(ReefBranch.K, new Transform2d(0, 0.014, Rotation2d.kZero)),
        // Map.entry(ReefBranch.L, new Transform2d(0, .03, Rotation2d.kZero)));

        // public static final Map<ReefBranch, Transform2d> scoringTransformsBlue =
        // Map.ofEntries(
        // Map.entry(ReefBranch.A, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.B, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.C, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.D, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.E, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.F, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.G, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.H, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.I, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.J, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.K, new Transform2d(0, 0, Rotation2d.kZero)),
        // Map.entry(ReefBranch.L, new Transform2d(0, 0, Rotation2d.kZero)));

        public static final Map<ReefBranch, Transform2d> troughScoringTransformsBlue = Map.ofEntries(
                Map.entry(ReefBranch.A, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.B, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.C, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.D, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.E, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.F, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.G, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.H, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.I, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.J, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.K, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.L, new Transform2d(0, 0, Rotation2d.kZero)));

        public static final Map<ReefBranch, Transform2d> troughScoringTransformsRed = Map.ofEntries(
                Map.entry(ReefBranch.A, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.B, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.C, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.D, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.E, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.F, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.G, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.H, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.I, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.J, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.K, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.L, new Transform2d(0, 0, Rotation2d.kZero)));

        // Competition
        public static final Map<ReefBranch, Transform2d> scoringTransformsRed = Map.ofEntries(
                Map.entry(ReefBranch.A, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.B, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.C, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.D, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.E, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.F, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.G, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.H, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.I, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.J, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.K, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.L, new Transform2d(0, 0, Rotation2d.kZero)));

        public static final Map<ReefBranch, Transform2d> scoringTransformsBlue = Map.ofEntries(
                Map.entry(ReefBranch.A, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.B, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.C, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.D, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.E, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.F, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.G, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.H, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.I, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.J, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.K, new Transform2d(0, 0, Rotation2d.kZero)),
                Map.entry(ReefBranch.L, new Transform2d(0, 0, Rotation2d.kZero)));

    }

    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0.0),
            ALGAE_GROUND_INTAKE_FOUR_BAR(0.4),
            ALGAE_INTAKE_CLEARED(0.6),

            ALGAE_INTAKE_L2(0.45 + Units.inchesToMeters(1)),
            ALGAE_INTAKE_L3(0.85 + Units.inchesToMeters(1)),
            BARGE_SCORE(1.56),
            AUTO_BARGE_SCORE(1.50),

            L1(0),
            L2(0.223 + 0.03),
            L3(0.64),
            L4(1.57 - Units.inchesToMeters(0.75)),
            TOP(1.56);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = 0.3;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int LEFT_MOTOR_ID = 11;
        public static final int RIGHT_MOTOR_ID = 12;
        public static final String MOTOR_CAN_BUS = "canivore";
        public static final InvertedValue LEFT_MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RIGHT_MOTOR_INVERSION = InvertedValue.CounterClockwise_Positive;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(2);
        public static final double GEARING = 3.0;
        public static final double MASS_KG = Units.lbsToKilograms(15);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(2.25) / 2.0; // 0.028575 m
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS; // 0.1795420202
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;
        public static final double DRUM_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.575; // TODO 1.575 Competition

        /**
         * The height at which the manipulator touches the top of the first stage,
         * making the first stage start moving.
         */
        public static final double STAGE_MOVEMENT_HEIGHT = 0.785;

        public static final int CURRENT_LIMIT = 80;

        public static final double kP = 28;
        public static final double kI = 0;
        public static final double kD = 1;
        public static final double kS = RobotBase.isReal() ? 0.165 : 0;
        public static final double kG = RobotBase.isReal() ? 0.345 : 0.53692;
        public static final double kG_HIGH = RobotBase.isReal() ? 0.465 : 0.53692;
        public static final double kV = 0.38;
        public static final double kA = 0.02143731721;
        public static final double TOLERANCE = 0.05;

        public static enum ElevatorProfileParams {
            SLOW(5, 5, 50),
            MID(5, 7, 70),
            FAST(5, 10, 100);

            public final double velocity;
            public final double acceleration;
            public final double jerk;

            private ElevatorProfileParams(double velocity, double acceleration, double jerk) {
                this.velocity = velocity;
                this.acceleration = acceleration;
                this.jerk = jerk;
            }
        }
    }

    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(-0.6806784083),
            CORAL_INTAKE(2.9321531434),
            HORIZONTAL(0),
            ALGAE_GROUND_INTAKE_FOUR_BAR(-0.75),
            ALGAE_GROUND_INTAKE(-0.21),
            ALGAE_REEF_INTAKE(0.3),
            BARGE_SCORE(1.1344640138),
            AUTO_BARGE_SCORE_START(0.9),
            AUTO_BARGE_SCORE(2.5),
            AUTO_BARGE_SCORE_RELEASE(1.05),
            CLIMB(0.934),
            END_CLIMB(1.5),
            ELEVATOR_SAFE(1.05),
            L1(0.457),
            L2(0.5), // reef angle
            L3(0.5),
            L4(-0.12),
            PROCESSOR(0.33),
            TOP(1.5707963268);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 13;
        public static final String MOTOR_CAN_BUS = "canivore";
        public static final InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
        public static final int ROLLER_MOTOR_ID = 14;
        public static final String ROLLER_MOTOR_CAN_BUS = "canivore";
        public static final InvertedValue ROLLER_MOTOR_INVERSION = InvertedValue.Clockwise_Positive;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60Foc(1);
        public static final double GEARING = 28.933333; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(6.25); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(10); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(-48);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(170);

        public static final int CURRENT_LIMIT = 100;
        public static final int ROLLER_CURRENT_LIMIT = 70;
        public static final int ROLLER_CURRENT_THRESHOLD = 10;

        // public static final double kP = 40; // TODO
        // public static final double kI = 0; // TODO
        // public static final double kD = 1; // TODO
        // // public static final double kS = 0.1;
        // public static final double kS = 0.0;
        // public static final double kS_CORAL = 0.0;
        // public static final double kS_ALGAE = 0.0;
        // // public static final double kG = 0.75;
        // public static final double kG = 0.37229; // sim only do not use
        // public static final double kG_CORAL = 0.37229; // TODO
        // public static final double kG_ALGAE = 0.37229; // TODO
        // public static final double kV = 1.4846;
        // public static final double kA = 0.039024;
        // public static final double kA = 0.18;

        public static final double kP = 40; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 1; // TODO
        public static final double kS = 0.08196429516;
        public static final double kG = 0.3642857563;
        public static final double kG_CORAL = 0.4098214758; // TODO
        public static final double kG_ALGAE = 0.4917857709; // TODO
        public static final double kV = 3.7169721208;
        public static final double kA = 0.1581615504;

        public static final double TOLERANCE = 0.1;

        public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 4;
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 16;
    }

    public static final class Intake {
        public static final int ROLLER_MOTOR_ID = 15;
        public static final String ROLLER_MOTOR_CAN_BUS = "rio";
        public static final InvertedValue ROLLER_MOTOR_INVERSION = InvertedValue.Clockwise_Positive;
        public static final int ROLLER_CURRENT_LIMIT = 60;

        public static enum IntakePosition {
            RETRACTED(Units.degreesToRadians(145)),
            CENTER(Math.PI / 2.0),
            EXTENDED(Units.degreesToRadians(40));

            public final double value;

            private IntakePosition(double value) {
                this.value = value;
            }
        }

        public static final int PIVOT_MOTOR_ID = 16;
        public static final String PIVOT_MOTOR_CAN_BUS = "canivore";
        public static final InvertedValue PIVOT_MOTOR_INVERSION = InvertedValue.Clockwise_Positive;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 33.0 / 11.0;
        public static final double MASS_KG = Units.lbsToKilograms(3);
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(38);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(143.81);

        public static final int CURRENT_LIMIT = 140;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 1; // TODO
        public static final double kS = 0; // TODO
        public static final double kG = 2.0; // TODO
        public static final double kV = 1.0;// TODO
        public static final double kA = 0.2;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_ROTATIONS_PER_SECOND = 2; // TODO
        public static final double MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED = 4; // TODO
    }

    public static final class Climber {
        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 10;
        public static final String MOTOR_CAN_BUS = "canivore";
        public static final InvertedValue MOTOR_INVERSION = InvertedValue.CounterClockwise_Positive;
        public static final int CURRENT_LIMIT = 63;

        public static final double MIN_POSITION_ROTATIONS = -3.4;
        public static final double MAX_POSITION_ROTATIONS = 0.0; // TODO

        public static final double GEARING = 9.0;
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;

    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            ELEVATOR_RIGHT(0, 53, false),
            ELEVATOR_L1_RIGHT(0, 13, false),
            ELEVATOR_L2_RIGHT(14, 26, false),
            ELEVATOR_L3_RIGHT(27, 40, false),
            ELEVATOR_L4_RIGHT(41, 53, false),
            ELEVATOR_CENTER(54, 94, false),
            ELEVATOR_CENTER_RIGHT(54, 74, false),
            ELEVATOR_CENTER_LEFT(75, 94, false),
            ELEVATOR_LEFT(95, 132, true),
            ELEVATOR_L1_LEFT(124, 132, true),
            ELEVATOR_L2_LEFT(114, 123, true),
            ELEVATOR_L3_LEFT(105, 113, true),
            ELEVATOR_L4_LEFT(95, 104, true),
            ALL(0, 132, true);

            private final int startIdx;
            private final int endIdx;
            private final boolean inverted;

            private LEDSection(int startIdx, int endIdx, boolean inverted) {
                this.startIdx = startIdx;
                this.endIdx = endIdx;
                this.inverted = inverted;
            }

            private LEDSection(int startIdx, int endIdx) {
                this(startIdx, endIdx, false);
            }

            @Override
            public int start() {
                return startIdx;
            }

            @Override
            public int end() {
                return endIdx;
            }

            @Override
            public boolean inverted() {
                return inverted;
            }

            @Override
            public int length() {
                return endIdx - startIdx + 1;
            }
        }

        public static final int PORT = 6;
        public static final int LENGTH = 133;

        public static final List<LEDState> DEFAULT_STATES = List.of(LEDState.FIRE);
    }

    public static final class Vision {
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Double.MAX_VALUE);
        public static final Matrix<N3, N1> MULTI_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.01, 0.01, Double.MAX_VALUE);
        // TODO
        public static final Matrix<N3, N1> SINGLE_TAG_PRECISE_STD_DEVS = VecBuilder.fill(0.05, 0.05, Double.MAX_VALUE);

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
                // left camera

                // old
                // new Transform3d(
                // new Translation3d(
                // 0.31603696, // 0.29845 m // difference is -0.0165896 m
                // 0.1679575, // 0.1905 m
                // 0.210531964),
                // new Rotation3d(0, Units.degreesToRadians(-5),
                // Units.degreesToRadians(-35))),
                new Transform3d(
                        new Translation3d(
                                0.29845, // 0.29845 m
                                0.1905, // 0.1905 m
                                0.210531964),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(-35))),
                // right camera
                new Transform3d(
                        new Translation3d(
                                0.29845,
                                -0.1905,
                                0.210531964),
                        new Rotation3d(0, Units.degreesToRadians(-20),
                                Units.degreesToRadians(35))),
                // unused back camera
                new Transform3d(
                        new Translation3d(
                                -0.31603696,
                                0.1679575,
                                0.210531964),
                        new Rotation3d(0, Units.degreesToRadians(-5),
                                Units.degreesToRadians(-110))),
        };
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 3.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.1;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_LIMIT = 0.8;
        public static final double AUTO_DRIVE_ADAPTIVE_SCALE_FACTOR = 0.5;
    }

    public static final class Auto {
        public static final Pose2d RIGHT_INTAKING_LOCATION = new Pose2d(1.3207684755325317, 0.7479230761528015,
                Rotation2d.fromDegrees(53));
        public static final Pose2d LEFT_INTAKING_LOCATION = new Pose2d(1.319039225578308, 7.206160545349121,
                Rotation2d.fromDegrees(-53));
    }
}

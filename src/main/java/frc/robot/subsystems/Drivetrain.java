package frc.robot.subsystems;

import java.util.Set;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import com.techhounds.houndutil.houndlib.swerve.KrakenCoaxialSwerveModule;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Teleop.*;

import static frc.robot.subsystems.Drivetrain.Constants.*;

@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    public static final class Constants {
        public static final class FrontLeft {
            public static final int DRIVE_MOTOR_ID = 6;
            public static final int STEER_MOTOR_ID = 5;
            public static final int STEER_ENCODER_ID = 0; // TODO
            public static final double STEER_ENCODER_OFFSET = 0.4521484375; // TODO
        }

        public static final class FrontRight {
            public static final int DRIVE_MOTOR_ID = 8;
            public static final int STEER_MOTOR_ID = 7;
            public static final int STEER_ENCODER_ID = 1; // TODO
            public static final double STEER_ENCODER_OFFSET = -0.179443359375 - 0.00634765625; // TODO
        }

        public static final class BackLeft {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int STEER_MOTOR_ID = 1;
            public static final int STEER_ENCODER_ID = 2; // TODO
            public static final double STEER_ENCODER_OFFSET = 0.242919921875; // TODO
        }

        public static final class BackRight {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int STEER_MOTOR_ID = 3;
            public static final int STEER_ENCODER_ID = 3; // TODO
            public static final double STEER_ENCODER_OFFSET = 0.498046875 - 0.003; // TODO
        }

        public static final String CAN_BUS = "canivore"; // TODO

        public static final int PIGEON_ID = 0; // TODO

        public static final boolean DRIVE_MOTORS_INVERTED = false; // TODO
        public static final boolean STEER_MOTORS_INVERTED = true; // TODO
        public static final boolean STEER_CANCODERS_INVERTED = RobotBase.isReal() ? false : true;

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.527; // TODO
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.527; // TODO
        public static final double DRIVE_BASE_RADIUS_METERS = 0.3727; // TODO

        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants();
        static {
            SWERVE_CONSTANTS.DRIVE_kP = 0.84992; // TODO
            SWERVE_CONSTANTS.DRIVE_kI = 0.0; // TODO
            SWERVE_CONSTANTS.DRIVE_kD = 0.0; // TODO
            SWERVE_CONSTANTS.DRIVE_kS = 0.2368; // TODO
            SWERVE_CONSTANTS.DRIVE_kV = 0.67229; // TODO
            SWERVE_CONSTANTS.DRIVE_kA = 0.080151; // TODO
            SWERVE_CONSTANTS.STEER_kP = 100.0; // TODO
            SWERVE_CONSTANTS.STEER_kI = 0.0; // TODO
            SWERVE_CONSTANTS.STEER_kD = 1.0; // TODO
            SWERVE_CONSTANTS.STEER_kS = 0; // TODO
            SWERVE_CONSTANTS.STEER_kV = 0; // TODO
            SWERVE_CONSTANTS.STEER_kA = 0; // TODO

            SWERVE_CONSTANTS.DRIVE_GEARING = 5.9; // Swerve Drive Specialties MK4n L2 ratio.
            SWERVE_CONSTANTS.STEER_GEARING = 18.75; // Swerve Drive Specialties MK4n ratio.
            SWERVE_CONSTANTS.COUPLING_RATIO = 50.0 / 16.0; // Inverse of swerve module first stage gear ratio.
            SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * 0.0491630791391; // TODO
            SWERVE_CONSTANTS.DRIVE_ENCODER_ROTATIONS_TO_METERS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE
                    / SWERVE_CONSTANTS.DRIVE_GEARING;
            SWERVE_CONSTANTS.STEER_ENCODER_ROTATIONS_TO_RADIANS = 2 * Math.PI
                    / SWERVE_CONSTANTS.STEER_GEARING;

            SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 4.54; // TODO
            SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8; // TODO
            SWERVE_CONSTANTS.MAX_STEER_VELOCITY_RADIANS_PER_SECOND = 100 * 2 * Math.PI; // TODO
            // max velocity in 1/3 sec
            SWERVE_CONSTANTS.MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 10 * 100 * 2 * Math.PI; // TODO

            SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT = 100; // TODO
            SWERVE_CONSTANTS.STEER_CURRENT_LIMIT = 30; // TODO
            SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.STEER_GEARBOX_REPR = DCMotor.getKrakenX60(1);
            SWERVE_CONSTANTS.DRIVE_MOI = 0.01; // TODO
            SWERVE_CONSTANTS.STEER_MOI = 0.025; // TODO
        }

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10; // TODO
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

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0; // TODO
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0; // TODO

        public static final double TRANSLATION_kP = 1.4; // TODO
        public static final double TRANSLATION_kI = 0; // TODO
        public static final double TRANSLATION_kD = 0.05; // TODO
        public static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                SWERVE_CONSTANTS.MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double ROTATION_kP = 1.3; // TODO
        public static final double ROTATION_kI = 0; // TODO
        public static final double ROTATION_kD = 0.05; // TODO
        public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final double MASS = 55.0; // KG, TODO
        public static final double MOMENT_OF_INERTIA = 4.9; // KG m^2, TODO
        public static final double SWERVE_WHEEL_RADIUS = SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE / (Math.PI * 2.0); // meters

        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(SWERVE_WHEEL_RADIUS,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND, 1.0, SWERVE_CONSTANTS.DRIVE_GEARBOX_REPR,
                SWERVE_CONSTANTS.DRIVE_CURRENT_LIMIT, 1);
        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(MASS, MOMENT_OF_INERTIA, MODULE_CONFIG,
                SWERVE_MODULE_LOCATIONS[0], SWERVE_MODULE_LOCATIONS[1], SWERVE_MODULE_LOCATIONS[2],
                SWERVE_MODULE_LOCATIONS[3]);

    }

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontLeft = new KrakenCoaxialSwerveModule(
            FrontLeft.DRIVE_MOTOR_ID,
            FrontLeft.STEER_MOTOR_ID,
            FrontLeft.STEER_ENCODER_ID,
            CAN_BUS,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FrontLeft.STEER_ENCODER_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule frontRight = new KrakenCoaxialSwerveModule(
            FrontRight.DRIVE_MOTOR_ID,
            FrontRight.STEER_MOTOR_ID,
            FrontRight.STEER_ENCODER_ID,
            CAN_BUS,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            FrontRight.STEER_ENCODER_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backLeft = new KrakenCoaxialSwerveModule(
            BackLeft.DRIVE_MOTOR_ID,
            BackLeft.STEER_MOTOR_ID,
            BackLeft.STEER_ENCODER_ID,
            CAN_BUS,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BackLeft.STEER_ENCODER_OFFSET,
            SWERVE_CONSTANTS);

    @Log(groups = "modules")
    private final KrakenCoaxialSwerveModule backRight = new KrakenCoaxialSwerveModule(
            BackRight.DRIVE_MOTOR_ID,
            BackRight.STEER_MOTOR_ID,
            BackRight.STEER_ENCODER_ID,
            CAN_BUS,
            DRIVE_MOTORS_INVERTED,
            STEER_MOTORS_INVERTED,
            STEER_CANCODERS_INVERTED,
            BackRight.STEER_ENCODER_OFFSET,
            SWERVE_CONSTANTS);

    @Log
    @SendableLog(name = "pigeonSendable")
    private final Pigeon2 pigeon = new Pigeon2(0, CAN_BUS);

    private SwerveDriveOdometry simOdometry;
    private SwerveModulePosition[] lastModulePositions = getModulePositions();

    // private final MutableMeasure sysidDriveAppliedVoltageMeasure =
    // Volts.mutable(0);
    // private final MutableMeasure sysidDrivePositionMeasure = Meters.mutable(0);
    // private final MutableMeasure sysidDriveVelocityMeasure =
    // MetersPerSecond.mutable(0);

    private Voltage sysidDriveAppliedVoltageMeasure = Volts.of(0);
    private Distance sysidDrivePositionMeasure = Meters.of(0);
    private LinearVelocity sysidDriveVelocityMeasure = MetersPerSecond.of(0);

    private final SysIdRoutine sysIdDrive;

    private Voltage sysidSteerAppliedVoltageMeasure = Volts.of(0);
    private Angle sysidSteerPositionMeasure = Rotations.of(0);
    private AngularVelocity sysidSteerVelocityMeasure = RotationsPerSecond.of(0);

    // private final MutableMeasure sysidSteerAppliedVoltageMeasure =
    // Volts.mutable(0);
    // private final MutableMeasure sysidSteerPositionMeasure =
    // Rotations.mutable(0);
    // private final MutableMeasure sysidSteerVelocityMeasure =
    // RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdSteer;

    private final Orchestra orchestra = new Orchestra();

    private final SwerveDrivePoseEstimator poseEstimator;

    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();

    private final OdometryThread odometryThread;

    @Log(groups = "control")
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD, TRANSLATION_CONSTRAINTS);

    @Log(groups = "control")
    private double driveToPoseDistance = 0.0;

    @Log(groups = "control")
    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            ROTATION_kP, ROTATION_kI, ROTATION_kD, ROTATION_CONSTRAINTS);

    @Log(groups = "control")
    private SwerveModuleState[] commandedModuleStates = new SwerveModuleState[] { new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState() };

    @Log(groups = "control")
    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds();

    private DriveMode driveMode = DriveMode.FIELD_ORIENTED;

    /**
     * Whether to override the inputs of the driver for maintaining or turning to a
     * specific angle.
     */
    @Log
    private boolean isControlledRotationEnabled = false;

    @Log
    private Pose2d targettedStagePose = new Pose2d();

    private ChassisSpeeds prevFieldRelVelocities = new ChassisSpeeds();

    @Log
    private double averageOdometryLoopTime = 0;
    @Log
    private int successfulDaqs = 0;
    @Log
    private int failedDaqs = 0;

    @Log
    private boolean initialized = false;

    /** Initializes the drivetrain. */
    public Drivetrain() {
        poseEstimator = new SwerveDrivePoseEstimator(
                KINEMATICS,
                getRotation(),
                getModulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        driveController.setTolerance(0.05);
        rotationController.setTolerance(0.05);
        rotationController.enableContinuousInput(0, 2 * Math.PI);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        if (RobotBase.isSimulation()) {
            simOdometry = new SwerveDriveOdometry(KINEMATICS, getRotation(), getModulePositions(), new Pose2d());
        }

        sysIdDrive = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("sysid_state", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() / 12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure = Volts
                                            .of(frontLeft.getDriveMotorVoltage()))
                                    .linearPosition(
                                            sysidDrivePositionMeasure = Meters.of(frontLeft.getDriveMotorPosition()))
                                    .linearVelocity(sysidDriveVelocityMeasure = MetersPerSecond
                                            .of(frontLeft.getDriveMotorVelocity()));
                            log.motor("frontRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure = Volts
                                            .of(frontRight.getDriveMotorVoltage()))
                                    .linearPosition(
                                            sysidDrivePositionMeasure = Meters.of(frontRight.getDriveMotorPosition()))
                                    .linearVelocity(sysidDriveVelocityMeasure = MetersPerSecond
                                            .of(frontRight.getDriveMotorVelocity()));
                            log.motor("backLeft")
                                    .voltage(sysidDriveAppliedVoltageMeasure = Volts
                                            .of(backLeft.getDriveMotorVoltage()))
                                    .linearPosition(
                                            sysidDrivePositionMeasure = Meters.of(backLeft.getDriveMotorPosition()))
                                    .linearVelocity(sysidDriveVelocityMeasure = MetersPerSecond
                                            .of(backLeft.getDriveMotorVelocity()));
                            log.motor("backRight")
                                    .voltage(sysidDriveAppliedVoltageMeasure = Volts
                                            .of(backRight.getDriveMotorVoltage()))
                                    .linearPosition(
                                            sysidDrivePositionMeasure = Meters.of(backRight.getDriveMotorPosition()))
                                    .linearVelocity(sysidDriveVelocityMeasure = MetersPerSecond
                                            .of(backRight.getDriveMotorVelocity()));
                        },
                        this));
        sysIdSteer = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drive(new ChassisSpeeds(
                                    SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND * volts.magnitude() /
                                            12.0,
                                    0,
                                    0));
                        },
                        log -> {
                            log.motor("frontLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure = Volts
                                            .of(frontLeft.getSteerMotorVoltage()))
                                    .angularPosition(sysidSteerPositionMeasure = Rotations
                                            .of(frontLeft.getSteerMotorPosition()))
                                    .angularVelocity(sysidSteerVelocityMeasure = RotationsPerSecond
                                            .of(frontLeft.getSteerMotorVelocity()));
                            log.motor("frontRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure = Volts
                                            .of(frontRight.getSteerMotorVoltage()))
                                    .angularPosition(sysidSteerPositionMeasure = Rotations
                                            .of(frontRight.getSteerMotorPosition()))
                                    .angularVelocity(sysidSteerVelocityMeasure = RotationsPerSecond
                                            .of(frontRight.getSteerMotorVelocity()));
                            log.motor("backLeft")
                                    .voltage(sysidSteerAppliedVoltageMeasure = Volts
                                            .of(backLeft.getSteerMotorVoltage()))
                                    .angularPosition(sysidSteerPositionMeasure = Rotations
                                            .of(backLeft.getSteerMotorPosition()))
                                    .angularVelocity(sysidSteerVelocityMeasure = RotationsPerSecond
                                            .of(backLeft.getSteerMotorVelocity()));
                            log.motor("backRight")
                                    .voltage(sysidSteerAppliedVoltageMeasure = Volts
                                            .of(backRight.getSteerMotorVoltage()))
                                    .angularPosition(sysidSteerPositionMeasure = Rotations
                                            .of(backRight.getSteerMotorPosition()))
                                    .angularVelocity(sysidSteerVelocityMeasure = RotationsPerSecond
                                            .of(backRight.getSteerMotorVelocity()));
                        },
                        this));

        orchestra.addInstrument(frontLeft.getDriveMotor(), 1);
        orchestra.addInstrument(frontRight.getDriveMotor(), 1);
        orchestra.addInstrument(backLeft.getDriveMotor(), 1);
        orchestra.addInstrument(backRight.getDriveMotor(), 1);
        orchestra.addInstrument(frontLeft.getSteerMotor(), 1);
        orchestra.addInstrument(frontRight.getSteerMotor(), 1);
        orchestra.addInstrument(backLeft.getSteerMotor(), 1);
        orchestra.addInstrument(backRight.getSteerMotor(), 1);

        odometryThread = new OdometryThread();
        odometryThread.start();

    }

    public class OdometryThread {
        // Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
        // If the odometry period is far away from the desired frequency, increasing
        // this may help
        private static final int START_THREAD_PRIORITY = 1;

        private final Thread m_thread;
        private volatile boolean m_running = false;

        private final BaseStatusSignal[] allSignals;

        private final MedianFilter peakRemover = new MedianFilter(3);
        private final LinearFilter lowPass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;

        private KrakenCoaxialSwerveModule[] modules = new KrakenCoaxialSwerveModule[] {
                frontLeft, frontRight, backLeft, backRight };
        private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        private int lastThreadPriority = START_THREAD_PRIORITY;
        private volatile int threadPriorityToSet = START_THREAD_PRIORITY;
        private final int UPDATE_FREQUENCY = 250;

        public OdometryThread() {
            m_thread = new Thread(this::run);
            /*
             * Mark this thread as a "daemon" (background) thread
             * so it doesn't hold up program shutdown
             */
            m_thread.setDaemon(true);

            /* 4 signals for each module + 2 for Pigeon2 */

            allSignals = new BaseStatusSignal[(4 * 4) + 2];
            for (int i = 0; i < 4; ++i) {
                BaseStatusSignal[] signals = modules[i].getSignals();
                allSignals[(i * 4) + 0] = signals[0];
                allSignals[(i * 4) + 1] = signals[1];
                allSignals[(i * 4) + 2] = signals[2];
                allSignals[(i * 4) + 3] = signals[3];
            }
            allSignals[allSignals.length - 2] = pigeon.getYaw();
            allSignals[allSignals.length - 1] = pigeon.getAngularVelocityZWorld();
        }

        /**
         * Starts the odometry thread.
         */
        public void start() {
            m_running = true;
            m_thread.start();
        }

        /**
         * Stops the odometry thread.
         */
        public void stop() {
            stop(0);
        }

        /**
         * Stops the odometry thread with a timeout.
         *
         * @param millis The time to wait in milliseconds
         */
        public void stop(long millis) {
            m_running = false;
            try {
                m_thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        /**
         * Runs the odometry thread.
         */
        public void run() {
            /* Make sure all signals update at the correct update frequency */
            BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY, allSignals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);

            /* Run as fast as possible, our signals will control the timing */
            while (m_running) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY, allSignals);

                try {
                    stateLock.writeLock().lock();

                    lastTime = currentTime;
                    currentTime = Timer.getFPGATimestamp();
                    /*
                     * We don't care about the peaks, as they correspond to GC events, and we want
                     * the period generally low passed
                     */
                    averageOdometryLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));

                    /* Get status of first element */
                    if (status.isOK()) {
                        successfulDaqs++;
                    } else {
                        failedDaqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < 4; ++i) {
                        modulePositions[i] = modules[i].getPosition();
                    }
                    double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                            pigeon.getYaw(), pigeon.getAngularVelocityZWorld()).in(Degrees);

                    /* Keep track of previous and current pose to account for the carpet vector */
                    poseEstimator.update(Rotation2d.fromDegrees(yawDegrees), modulePositions);
                    if (RobotBase.isSimulation()) {
                        simOdometry.update(getRotation(), getModulePositions());
                    }
                } finally {
                    stateLock.writeLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (threadPriorityToSet != lastThreadPriority) {
                    Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                    lastThreadPriority = threadPriorityToSet;
                }
            }
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified
         * priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher
         *                 priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            threadPriorityToSet = priority;
        }
    }

    @Override
    public void periodic() {
        prevFieldRelVelocities = getFieldRelativeSpeeds();
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    /**
     * Updates simulation-specific variables.
     */
    @Override
    public void simulationPeriodic() {
        SwerveModulePosition[] currentPositions = getModulePositions();
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            deltas[i] = new SwerveModulePosition(
                    currentPositions[i].distanceMeters - lastModulePositions[i].distanceMeters,
                    currentPositions[i].angle);
        }

        pigeon.getSimState().setRawYaw(pigeon.getYaw().getValueAsDouble() +
                Units.radiansToDegrees(KINEMATICS.toTwist2d(deltas).dtheta));

        lastModulePositions = currentPositions;
    }

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    @Override
    @Log
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Log
    public Pose2d getSimPose() {
        if (simOdometry != null)
            return simOdometry.getPoseMeters();
        else
            return new Pose2d();
    }

    @Override
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    @Override
    @Log(groups = "control")
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    @Override
    @Log(groups = "control")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    @Override
    @Log(groups = "control")
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @Log(groups = "control")
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(KINEMATICS.toChassisSpeeds(getModuleStates()), getRotation());
    }

    @Log(groups = "control")
    public ChassisAccelerations getFieldRelativeAccelerations() {
        return new ChassisAccelerations(getFieldRelativeSpeeds(), prevFieldRelVelocities, 0.020);
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters    The pose of the robot from vision.
     * @param timestampSeconds         The timestamp of the vision measurement.
     * @param visionMeasurementStdDevs The standard deviations of the vision
     *                                 measurement.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            stateLock.writeLock().lock();
            poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            stateLock.writeLock().unlock();
        }
    }

    @Override
    public void updatePoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstimator'");
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getModulePositions(),
                new Pose2d(pose.getTranslation(), getRotation()));
        if (RobotBase.isSimulation())
            simOdometry.resetPosition(getRotation(), getModulePositions(),
                    new Pose2d(pose.getTranslation(), getRotation()));
    }

    @Override
    public void resetGyro() {
        pigeon.setYaw(0);
        initialized = true;
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        frontLeft.setMotorHoldMode(motorHoldMode);
        frontRight.setMotorHoldMode(motorHoldMode);
        backLeft.setMotorHoldMode(motorHoldMode);
        backRight.setMotorHoldMode(motorHoldMode);
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        frontLeft.setDriveCurrentLimit(currentLimit);
        frontRight.setDriveCurrentLimit(currentLimit);
        backLeft.setDriveCurrentLimit(currentLimit);
        backRight.setDriveCurrentLimit(currentLimit);
    }

    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    @Override
    public void setStates(SwerveModuleState[] states) {
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] states) {
        frontLeft.setStateClosedLoop(states[0]);
        frontRight.setStateClosedLoop(states[1]);
        backLeft.setStateClosedLoop(states[2]);
        backRight.setStateClosedLoop(states[3]);
    }

    /**
     * Drives the robot with the given chassis speeds.
     *
     * @param speeds The desired chassis speeds.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, this.driveMode);
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        // compensates for swerve skew
        adjustedChassisSpeeds = ChassisSpeeds.discretize(adjustedChassisSpeeds, 0.02);
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states = new SwerveModuleState[] {
                SwerveModuleState.optimize(states[0], frontLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[1], frontRight.getWheelAngle()),
                SwerveModuleState.optimize(states[2], backLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[3], backRight.getWheelAngle()),
        };

        commandedModuleStates = states;
        setStates(states);
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                && driveMode == DriveMode.FIELD_ORIENTED) {
            speeds.vxMetersPerSecond *= -1;
            speeds.vyMetersPerSecond *= -1;
        }

        commandedChassisSpeeds = speeds;
        ChassisSpeeds adjustedChassisSpeeds = null;
        switch (driveMode) {
            case ROBOT_RELATIVE:
                adjustedChassisSpeeds = speeds;
                break;
            case FIELD_ORIENTED:
                adjustedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, getRotation());
                break;
        }

        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(adjustedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states,
                SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        states = new SwerveModuleState[] {
                SwerveModuleState.optimize(states[0], frontLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[1], frontRight.getWheelAngle()),
                SwerveModuleState.optimize(states[2], backLeft.getWheelAngle()),
                SwerveModuleState.optimize(states[3], backRight.getWheelAngle()),
        };

        commandedModuleStates = states;
        setStatesClosedLoop(states);
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter thetaSpeedLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double xSpeed = xSpeedSupplier.getAsDouble();
            double ySpeed = ySpeedSupplier.getAsDouble();
            double thetaSpeed = thetaSpeedSupplier.getAsDouble();

            xSpeed = MathUtil.applyDeadband(xSpeed, JOYSTICK_INPUT_DEADBAND);
            ySpeed = MathUtil.applyDeadband(ySpeed, JOYSTICK_INPUT_DEADBAND);
            thetaSpeed = MathUtil.applyDeadband(thetaSpeed, JOYSTICK_INPUT_DEADBAND);

            xSpeed = Math.copySign(Math.pow(xSpeed, JOYSTICK_CURVE_EXP), xSpeed);
            ySpeed = Math.copySign(Math.pow(ySpeed, JOYSTICK_CURVE_EXP), ySpeed);
            thetaSpeed = Math.copySign(Math.pow(thetaSpeed, JOYSTICK_ROT_CURVE_EXP), thetaSpeed);

            xSpeed = xSpeedLimiter.calculate(xSpeed);
            ySpeed = ySpeedLimiter.calculate(ySpeed);
            thetaSpeed = thetaSpeedLimiter.calculate(thetaSpeed);

            if (isControlledRotationEnabled) {
                thetaSpeed = rotationController.calculate(getRotation().getRadians());
            }

            // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
            // physical velocity to output in m/s.
            xSpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            ySpeed *= SWERVE_CONSTANTS.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            thetaSpeed *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;

            drive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed), driveMode);
        }).withName("drivetrain.teleopDrive");
    }

    @Override
    public Command controlledRotateCommand(DoubleSupplier angle) {
        return Commands.run(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getRotation().getRadians());
            }
            isControlledRotationEnabled = true;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).withName("drivetrain.controlledRotate");
    }

    /**
     * Same as controlledRotateCommand but includes the run command to actually
     * execute the rotation
     *
     * @param angle The desired angle to rotate to.
     * @return The command to rotate the robot.
     */
    public Command standaloneControlledRotateCommand(DoubleSupplier angle) {
        return runOnce(() -> {
            if (!isControlledRotationEnabled) {
                rotationController.reset(getRotation().getRadians());
            }
            // isControlledRotationEnabled = true;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
                rotationController.setGoal(angle.getAsDouble() + Math.PI);
            else
                rotationController.setGoal(angle.getAsDouble());
        }).andThen(run(() -> {
            drive(new ChassisSpeeds(0, 0,
                    rotationController.calculate(poseEstimator.getEstimatedPosition().getRotation().getRadians())),
                    driveMode);
        })).withName("drivetrain.standaloneControlledRotate");
    }

    @Override
    public Command disableControlledRotateCommand() {
        return Commands.runOnce(
                () -> {
                    isControlledRotationEnabled = false;
                }).withName("drivetrain.disableControlledRotate");
    }

    @Override
    public Command wheelLockCommand() {
        return run(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        }).withName("drivetrain.wheelLock");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        return Commands.runOnce(() -> {
            setStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle)),
                    new SwerveModuleState(0, new Rotation2d(angle))

            });
        }).withName("drivetrain.turnWheelsToAngle");
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return runOnce(() -> {
            driveController.reset(getPose().getTranslation().getDistance(poseSupplier.get().getTranslation()));
            rotationController.reset(getPose().getRotation().getRadians());
        }).andThen(run(() -> {
            driveToPoseDistance = getPose().getTranslation().getDistance(poseSupplier.get().getTranslation());
            double driveVelocityScalar = driveController.getSetpoint().velocity + driveController.calculate(
                    driveToPoseDistance, 0.0);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                driveVelocityScalar *= -1;
            }
            if (driveController.atGoal())
                driveVelocityScalar = 0.0;

            Translation2d driveVelocity = new Pose2d(
                    new Translation2d(),
                    getPose().getTranslation().minus(poseSupplier.get().getTranslation()).getAngle())
                    .transformBy(new Transform2d(driveVelocityScalar, 0, new Rotation2d()))
                    .getTranslation();

            drive(
                    new ChassisSpeeds(
                            driveVelocity.getX(),
                            driveVelocity.getY(),
                            rotationController.calculate(getPose().getRotation().getRadians(),
                                    poseSupplier.get().getRotation().getRadians())),
                    DriveMode.FIELD_ORIENTED);
        })).withName("drivetrain.driveToPose");
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                this::getPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> driveClosedLoop(speeds, DriveMode.ROBOT_RELATIVE),
                new PPHolonomicDriveController(
                        new PIDConstants(PATH_FOLLOWING_TRANSLATION_kP, 0, 0),
                        new PIDConstants(PATH_FOLLOWING_ROTATION_kP, 0, 0)),
                ROBOT_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this).finallyDo(this::stop).withName("drivetrain.followPath");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        return new DeferredCommand(() -> followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(
                                getPose(), getPose().plus(delta)),
                        constraints, new IdealStartingState(0, delta.getRotation().plus(getRotation())),
                        new GoalEndState(0, delta.getRotation().plus(getRotation())))),
                Set.of()).withName("drivetrain.driveDelta");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        return runOnce(() -> this.driveMode = driveMode).withName("drivetrain.setDriveMode");
    }

    @Override
    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro()).withName("drivetrain.resetGyro");
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> {
            frontLeft.setDriveCurrentLimit(currentLimit);
            frontRight.setDriveCurrentLimit(currentLimit);
            backLeft.setDriveCurrentLimit(currentLimit);
            backRight.setDriveCurrentLimit(currentLimit);
        }).withName("drivetrain.setDriveCurrentLimit");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(this::stop)
                .andThen(() -> setMotorHoldModes(MotorHoldMode.COAST))
                .finallyDo((d) -> setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("drivetrain.coastMotors");
    }

    /**
     * Draws the robot on a Field2d. This will include the angles of the swerve
     * modules on the outsides of the robot box in Glass.
     * 
     * @param field the field to draw the robot on (usually
     *              {@code AutoManager.getInstance().getField()})
     */
    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        if (RobotBase.isSimulation())
            field.getObject("simPose").setPose(simOdometry.getPoseMeters());

        // Draw a pose that is based on the robot pose, but shifted by the
        // translation of the module relative to robot center,
        // then rotated around its own center by the angle of the module.
        // field.getObject("modules").setPoses(
        // getPose().transformBy(
        // new Transform2d(SWERVE_MODULE_LOCATIONS[0],
        // getModulePositions()[0].angle)),
        // getPose().transformBy(
        // new Transform2d(SWERVE_MODULE_LOCATIONS[1],
        // getModulePositions()[1].angle)),
        // getPose().transformBy(
        // new Transform2d(SWERVE_MODULE_LOCATIONS[2],
        // getModulePositions()[2].angle)),
        // getPose().transformBy(
        // new Transform2d(SWERVE_MODULE_LOCATIONS[3],
        // getModulePositions()[3].angle)));
    }

    /**
     * Creates a command to perform a quasistatic system identification on the drive
     * motors.
     *
     * @param direction The direction to perform the identification in.
     * @return The command to perform the identification.
     */
    public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdDrive.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    /**
     * Creates a command to perform a dynamic system identification on the drive
     * motors.
     *
     * @param direction The direction to perform the identification in.
     * @return The command to perform the identification.
     */
    public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
        return sysIdDrive.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    /**
     * Creates a command to perform a quasistatic system identification on the steer
     * motors.
     *
     * @param direction The direction to perform the identification in.
     * @return The command to perform the identification.
     */
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdSteer.quasistatic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    /**
     * Creates a command to perform a dynamic system identification on the steer
     * motors.
     *
     * @param direction The direction to perform the identification in.
     * @return The command to perform the identification.
     */
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return sysIdSteer.dynamic(direction).withName("drivetrain.sysIdDriveQuasistatic");
    }

    /**
     * Returns whether the drivetrain has been initialized.
     *
     * @return True if the drivetrain has been initialized, false otherwise.
     */
    public boolean getInitialized() {
        return initialized;
    }

    /**
     * Creates a command to set the initialized state of the drivetrain.
     *
     * @param initialized The desired initialized state.
     * @return The command to set the initialized state.
     */
    public Command setInitializedCommand(boolean initialized) {
        return Commands.runOnce(() -> {
            this.initialized = initialized;
        }).withName("drivetrain.setInitialized");
    }
}

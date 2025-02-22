package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator.Constants.CAN;
import frc.robot.subsystems.Elevator.Constants.Feedback;
import frc.robot.subsystems.Elevator.Constants.Feedforward;
import frc.robot.subsystems.Elevator.Constants.MotionProfile;
import frc.robot.subsystems.Elevator.Constants.Position;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

import static frc.robot.subsystems.Elevator.Constants.*;

/** Subsystem which lifts manipulator and manipulator pivot. */
@LoggedObject
public class Elevator extends SubsystemBase implements BaseLinearMechanism<Position> {
    /** Constant values of elevator. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * elevator motors to be rotation which pulls elevator up away from zero point.
         * Left motor direction is opposite of right motor direction.
         */
        public static final InvertedValue LEFT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * elevator motors to be rotation which pulls elevator up away from zero point.
         * Right motor direction is opposite of left motor direction.
         */
        public static final InvertedValue RIGHT_MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive; // TODO
        /** Ratio of motor rotations to spool drum rotations. */
        public static final double GEAR_RATIO = 3 / 1;
        /** Radius of elevator spool drum. */
        public static final double DRUM_RADIUS = Units.inchesToMeters(1.05);
        /** Circumference of elevator spool drum. */
        public static final double DRUM_CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
        /** Ratio of motor rotations to spool drum rotations. */
        public static final double SENSOR_TO_MECHANISM = GEAR_RATIO;
        /** Current limit of elevator motors, in amps. */
        public static final double CURRENT_LIMIT = 80;
        public static final double POSITION_TOLERANCE = 0.1;
        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(2);
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(2.5) / 2.0; // 0.03175 m
        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.565; // TODO
        /**
         * The height at which the manipulator touches the top of the first stage,
         * making the first stage start moving.
         */
        public static final double STAGE_MOVEMENT_HEIGHT = 0.785;

        /** CAN information of elevator motors. */
        public static final class CAN {
            /** CAN bus elevator motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of elevator motors. */
            public static final class IDs {
                /** CAN ID of elevator left motor. */
                private static final int LEFT_MOTOR = 11;
                /** CAN ID of elevator right motor. */
                private static final int RIGHT_MOTOR = 12;
            }
        }

        /** Positions elevator can be in, in spool drum rotations */
        public static enum Position {
            HARD_STOP(0),
            GROUND_ALGAE(0), // TODO
            PROCESSOR(0.0), // TODO get actual position
            L1(0.0), // TODO get actual position
            L2(2.95), // TODO get actual position
            REEF_LOW_ALGAE(3.95),
            L3(5.25), // TODO get actual position
            REEF_HIGH_ALGAE(6.95),
            L4_NET(8.8);

            public final double position;

            private Position(final double position) {
                this.position = position;
            }
        }

        /**
         * Constants for feedforward control for moving to position setpoints.
         */
        public static final class Feedforward {
            /** Voltage required to overcome gravity. */
            public static final double kG = 0.3; // TODO find good value
            /** Voltage required to overcome motor's static friction. */
            public static final double kS = 0.2; // TODO find good value
            /** Voltage required to maintain constant velocity on motor. */
            public static final double kV = 2.22 * DRUM_CIRCUMFERENCE; // TODO find good value
            /** Voltage required to induce a given acceleration on motor. */
            public static final double kA = 0.05 * DRUM_CIRCUMFERENCE; // TODO find good value
        }

        /**
         * Constants for PID feedback control for error correction for moving to
         * position setpoints.
         */
        public static final class Feedback {
            /** Proportional term constant which drives error to zero proportionally. */
            public static final double kP = 50; // TODO find good value
            /**
             * Integral term constant which overcomes steady-state error. Should be used
             * with caution due to integral windup.
             */
            public static final double kI = 0; // TODO find good value
            /** Derivative term constant which dampens rate of error correction. */
            public static final double kD = 0; // TODO find good value
        }

        /**
         * Constants for CTRE's Motion Magic motion profiling for moving to position
         * setpoints with consistent and smooth motion across entire course of motion.
         */
        public static final class MotionProfile {
            /** Target cruise velocity along course of motion. */
            public static final double CRUISE_VELOCITY = 2; // 90
            /** Target acceleration of beginning and end of course of motion. */
            public static final double ACCELERATION = 2; // 80
            /** Target jerk along course of motion. */
            public static final double JERK = 0; // TODO
        }
    }

    /** Elevator left motor. */
    @Log
    private final TalonFX leftMotor = new TalonFX(CAN.IDs.LEFT_MOTOR, CAN.BUS);
    /** Elevator right motor. */
    @Log
    private final TalonFX rightMotor = new TalonFX(CAN.IDs.RIGHT_MOTOR, CAN.BUS);
    /**
     * Configuration object for configurations shared across both elevator motors.
     */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(Position.HARD_STOP.position);
    /** Request object for setting elevator motors' voltage directly. */
    private final VoltageOut voltageRequest = new VoltageOut(0);
    /** Request object for stopping the motors. */
    private final NeutralOut stopRequest = new NeutralOut();

    /**
     * Global position tracker object for tracking mechanism positions for
     * calculating safeties.
     */
    private final PositionTracker positionTracker;

    /**
     * Whether elevator position has been reset while the elevator is resting
     * against its hard stop using. If uninitialized, the mechanism should not be
     * able to move at all. Initialized to {@code false} until position is reset by
     * {@link frc.robot.HoundBrian HoundBrian}.
     */
    @Log
    private boolean initialized = false;

    /**
     * SysId routine to run to empirically determine feedforward and feedback
     * constant values, using CTRE's Phoenix 6 {@link SignalLogger Signal Logger}.
     */
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3), null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> {
                setVoltage(voltage.magnitude());
            }, null, this));

    private final ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEAR_RATIO,
            MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            false,
            Position.HARD_STOP.position);

    /**
     * Initialize elevator configurations and add pivot position to global position
     * tracker.
     * 
     * @param positionTracker global position tracker object
     */
    public Elevator(PositionTracker positionTracker) {
        motorConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        motorConfigs.MotorOutput.Inverted = LEFT_MOTOR_DIRECTION;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        motorConfigs.Slot0.kG = Feedforward.kG;
        motorConfigs.Slot0.kS = Feedforward.kS;
        motorConfigs.Slot0.kV = Feedforward.kV;
        motorConfigs.Slot0.kA = Feedforward.kA;
        motorConfigs.Slot0.kP = Feedback.kP;
        motorConfigs.Slot0.kI = Feedback.kI;
        motorConfigs.Slot0.kD = Feedback.kD;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = MotionProfile.CRUISE_VELOCITY;
        motorConfigs.MotionMagic.MotionMagicAcceleration = MotionProfile.ACCELERATION;
        motorConfigs.MotionMagic.MotionMagicJerk = MotionProfile.JERK;

        leftMotor.getConfigurator().apply(motorConfigs);
        rightMotor.getConfigurator().apply(motorConfigs);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        this.positionTracker = positionTracker;
        positionTracker.addPositionSupplier("Elevator", this::getPosition);

        // setDefaultCommand(holdCurrentPositionCommand());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = leftMotor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        elevatorSim.setInputVoltage(motorVoltage.in(Volts));
        elevatorSim.update(0.020);

        // set positions of the rotors by working back through units
        talonFXSim.setRawRotorPosition(elevatorSim.getPositionMeters() / DRUM_CIRCUMFERENCE * GEAR_RATIO);
        talonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() / DRUM_CIRCUMFERENCE * GEAR_RATIO);
    }

    @Log(groups = "components")
    public Pose3d getStageComponentPose() {
        Transform3d transform = new Transform3d();
        if (getPosition() > STAGE_MOVEMENT_HEIGHT) {
            transform = new Transform3d(0, 0, getPosition() - STAGE_MOVEMENT_HEIGHT, new Rotation3d());
        }
        return new Pose3d(0.1747, 0, 0.0417, new Rotation3d()).plus(transform);
    }

    @Log(groups = "components")
    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.1747, 0, 0.0671 + getPosition(), new Rotation3d());
    }

    /**
     * Determines if a desired control request for the motor(s) is safe for the
     * mechanism or not, and stops the motor(s) if unsafe. Unsafe control requests
     * include any requests when the mechanism is uninitialized, if the request will
     * cause the mechanism to exceed its physical limits, or if the request will
     * interfere with and break another mechanism.
     * 
     * @param controlRequest desired control request
     * @return desired control request if safe, else the control request to stop the
     *         motor.
     */
    public ControlRequest controlRequestWithSafeties(ControlRequest controlRequest) {
        if (!initialized) {
            return stopRequest;
        }

        // if (getPosition() > Position.L3.position && getPosition() <=
        // Position.L4_NET.position) {
        // return controlRequest;
        // }

        if (positionTracker.getPosition("Pivot") >= Pivot.Constants.Position.PAST_ELEVATOR.position) {
            return stopRequest;
        }

        // if (getPosition() > Position.L4_NET.position) {
        // return stopRequest;
        // }

        return controlRequest;
    }

    @Log
    public boolean atGoal() {
        return Math.abs(getPosition() - motionMagicVoltageRequest.Position) < POSITION_TOLERANCE;
    }

    @Override
    @Log
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        leftMotor.setPosition(Position.HARD_STOP.position);
        rightMotor.setPosition(Position.HARD_STOP.position);
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setControl(
                controlRequestWithSafeties(voltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12))));
    }

    // @Override
    // public Command moveToCurrentGoalCommand() {
    // return run(() -> leftMotor.setControl(controlRequestWithSafeties(
    // motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position).withEnableFOC(true))))
    // .until(this::atGoal).withName("elevator.moveToCurrentGoalCommand");
    // }
    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            double currentPosition = getPosition();
            double targetPosition = motionMagicVoltageRequest.Position;
            boolean atTarget = Math.abs(currentPosition - targetPosition) <= 0.05;

            System.out.println("Elevator Current: " + currentPosition + ", Target: " + targetPosition
                    + ", At Target: " + atTarget);

            if (!atTarget) {
                leftMotor.setControl(controlRequestWithSafeties(
                        motionMagicVoltageRequest.withPosition(targetPosition).withEnableFOC(true)));
            }
        }).until(() -> Math.abs(getPosition() - motionMagicVoltageRequest.Position) <= 0.05)
                .andThen(runOnce(() -> {
                    System.out.println("Elevator Reached goal position.");
                }))
                .withName("elevator.moveToCurrentGoalCommand");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
                .withName("elevator.moveToPositionCommand");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    double targetPosition = goalPositionSupplier.get();
                    System.out.println("Elevator Moving to position: " + targetPosition);
                    leftMotor.setControl(
                            controlRequestWithSafeties(motionMagicVoltageRequest.withPosition(targetPosition)
                                    .withEnableFOC(true)));
                }),
                moveToCurrentGoalCommand()
                        .andThen(runOnce(() -> System.out.println("Elevator Reached goal position."))))
                .withName("elevator.moveToArbitraryPositionCommand");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(
                () -> (motionMagicVoltageRequest.Position + delta.get()))
                .withName("elevator.movePositionDeltaCommand");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> {
            leftMotor.setControl(controlRequestWithSafeties(motionMagicVoltageRequest.withPosition(getPosition())));
        }).withName("elevator.holdCurrentPositionCommand");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> {
            resetPosition();
            initialized = true;
        }).withName("elevator.resetPositionCommand");

    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> {
            setVoltage(speed.get() * 12.0);
        }).withName("elevator.setOverridenSpeedCommand");

    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }).andThen(() -> {
            leftMotor.setNeutralMode(NeutralModeValue.Coast);
            rightMotor.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            leftMotor.setNeutralMode(NeutralModeValue.Brake);
            rightMotor.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("elevator.coastMotorsCommand");
    }

    /**
     * Creates a command to run a SysId quasistatic test in the specified direction,
     * which gradually ramps up voltage fed to mechanism in such a way as to
     * eliminate the effect of voltage on the mechanism's acceleration.
     * 
     * @param direction direction to run test in
     * @return the command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Creates a command to run a SysId dynamic test in the specified direction,
     * which steps up voltage fed to mechanism by a constant value to determine
     * mechanism behavior while accelerating.
     * 
     * @param direction direction to run test in
     * @return the command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
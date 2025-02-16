package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
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
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import static frc.robot.subsystems.Elevator.Constants.*;

/** Subsystem which lifts manipulator and manipulator pivot. */
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
        public static final InvertedValue RIGHT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
        /** Ratio of motor rotations to elevator spool drum rotations. */
        public static final double GEAR_RATIO = 3 / 1;
        /** Radius of elevator spool drum. */
        public static final double DRUM_RADIUS = Units.inchesToMeters(1.05);
        /** Circumference of elevator spool drum. */
        public static final double DRUM_CIRCUMFERENCE = 2 * Math.PI * DRUM_RADIUS;
        /** Ratio of motor rotations to elevator height in meters. */
        public static final double SENSOR_TO_MECHANISM = GEAR_RATIO;
        /** Current limit of elevator motors. */
        public static final double CURRENT_LIMIT = 80; // TODO

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

        /** Positions elevator can be in. */
        public static enum Position {
            HARD_STOP(0),
            ZERO(0.0), // TODO get actual position
            GROUND(0.0), // TODO get actual position
            L1(0.0), // TODO get actual position
            L2(0.0), // TODO get actual position
            L3(0.0), // TODO get actual position
            L4(0.0), // TODO get actual positions
            PROCESSOR(0.0), // TODO get actual position
            SOFT_STOP(1.455);

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
            public static final double kS = 0.45; // TODO find good value
            /** Voltage required to maintain constant velocity on motor. */
            public static final double kV = 2.22 / DRUM_CIRCUMFERENCE; // TODO find good value
            /** Voltage required to induce a given acceleration on motor. */
            public static final double kA = 0.05 / DRUM_CIRCUMFERENCE; // TODO find good value
        }

        /**
         * Constants for PID feedback control for error correction for moving to
         * position setpoints.
         */
        public static final class Feedback {
            /** Proportional term constant which drives error to zero proportionally. */
            public static final double kP = 0.25; // TODO find good value
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
            public static final double CRUISE_VELOCITY = 0.25; // TODO
            /** Target acceleration of beginning and end of course of motion. */
            public static final double ACCELERATION = 0.25; // TODO
            /** Target jerk along course of motion. */
            public static final double JERK = 0; // TODO
        }
    }

    /** Elevator left motor. */
    private final TalonFX leftMotor = new TalonFX(CAN.IDs.LEFT_MOTOR, CAN.BUS);
    /** Elevator right motor. */
    private final TalonFX rightMotor = new TalonFX(CAN.IDs.RIGHT_MOTOR, CAN.BUS);
    /**
     * Configuration object for configurations shared across both elevator motors.
     */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0);
    /** Request object for setting elevator motors voltage directly. */
    private final VoltageOut directVoltageRequest = new VoltageOut(0);
    private final NeutralOut stop = new NeutralOut();

    /** Mutable measure for voltages applied during sysId testing */
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    /** Mutable measure for distances traveled during sysId testing (Meters) */
    private final MutDistance sysIdDistance = Meters.mutable(0);
    /** Mutable measure for velocity during SysId testing (Meters/Second) */
    private final MutLinearVelocity sysIdVelocity = MetersPerSecond.mutable(0);
    /**
     * The sysIdRoutine object with default configuration and logging of voltage,
     * velocity, and distance
     */
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(3), null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> {
                setVoltage(voltage.magnitude());
            }, log -> {
                log.motor("Elevator")
                        .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .linearPosition(sysIdDistance.mut_replace(getPosition(), Meters))
                        .linearVelocity(sysIdVelocity.mut_replace(getVelocity(), MetersPerSecond));
            }, this));

    public Elevator() {
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

        setDefaultCommand(moveToArbitraryPositionCommand(() -> 0.01));
    }

    /**
     * Finds the position of the {@link #leftMotor motor}
     * 
     * @return The position of the elevator (in whatever unit is used for the
     *         {@link Constants#SENSOR_TO_MECHANISM conversion factor})
     */
    @Override
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Finds the voltage of the {@link #leftMotor motor}
     * 
     * @return Voltage of the motor, as a double
     */
    public double getVoltage() {
        return leftMotor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Finds the velocity of the mechanism controlled by the {@link #leftMotor
     * motor}
     * 
     * @return Velocity of the mechanism, in
     *         {@link edu.wpi.first.units.Units#MetersPerSecond Meters per Second}
     */
    public double getVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Resets the position of the motors to a specified {@link Constants#HARD_STOP
     * position} (Should be a hard stop)
     */
    @Override
    public void resetPosition() {
        leftMotor.setPosition(Position.ZERO.position);
        rightMotor.setPosition(Position.ZERO.position);
    }

    /**
     * Sets the voltage of the motors directly, {@link MathUtil#clamp clamped} to
     * safe values
     */
    @Override
    public void setVoltage(double voltage) {
        leftMotor.setControl(directVoltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
    }

    public boolean shouldEnforceSafeties(double currentPosition, double goalPosition) {
        boolean positiveTravel = goalPosition - currentPosition > 0;

        if (positiveTravel) {
            return currentPosition >= Position.SOFT_STOP.position;
        } else {
            return currentPosition <= Position.HARD_STOP.position;
        }
    }

    /**
     * Moves the motors to the currently set goal
     * 
     * @return Command to move the motors to the already set goal
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        // return run(() -> {
        // if (shouldEnforceSafeties(getPosition(), motionMagicVoltageRequest.Position))
        // {
        // leftMotor.setControl(stop);
        // } else {
        // leftMotor.setControl(motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position)
        // .withEnableFOC(true));
        // }
        // }).withName("elevator.moveToCurrentGoalCommand");

        return run(() -> leftMotor.setControl(
                motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position / DRUM_CIRCUMFERENCE)
                        .withEnableFOC(true)));
    }

    /**
     * Moves the motors to one of the main {@link Position positions}
     * 
     * @return Command that will move the motors to the setpoint enum
     */
    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
                .withName("elevator.moveToPositionCommand");
    }

    /**
     * Moves the motors to any position supplied
     * 
     * @return Command that will move the motors to the position supplied
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(runOnce(() -> {
            leftMotor
                    .setControl(motionMagicVoltageRequest.withPosition(goalPositionSupplier.get() / DRUM_CIRCUMFERENCE)
                            .withEnableFOC(true));
        }), moveToCurrentGoalCommand()).withName("elevator.moveToArbitraryPositionCommand");
    }

    /**
     * Moves the elevator's motors to a position relative to the current position.
     * 
     * @param delta A supplier of how much relative change is wanted, in meters.
     *              (Positive moves up)
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> motionMagicVoltageRequest.Position + delta.get())
                .withName("elevator.movePositionDeltaCommand");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> {
            leftMotor.setControl(motionMagicVoltageRequest.withPosition(getPosition()));
        })
                .withName("elevator.holdCurrentPositionCommand");
    }

    /**
     * Creates an instantaneous command that resets the position of the elevator
     * mechanism.
     * 
     * @return command to reset position
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition())
                .withName("elevator.resetPositionCommand");

    }

    /**
     * Creates a command that will manually set the speed of the elevator's motors
     * 
     * @param speed Supplier for the speed (-1 to 1) to set the motor to
     * @return Command that sets the speed to the value returned by the supplier
     */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> {
            setVoltage(speed.get() * 12.0);
        })
                .withName("elevator.setOverridenSpeedCommand");

    }

    /**
     * Creates command to stop the elevator's motors and allow them to coast.
     * 
     * @return the command that allows motors to coast
     */
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
     * Creates a command for the sysId quasistatic test, which gradually speeds up
     * the mechanism to eliminate variation from acceleration
     * 
     * @param direction Direction to run the motors in
     * @return Command that runs the quasistatic test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Creates a command for the sysId dynamic test, which will step up the speed to
     * see how the mechanism behaves during acceleration
     * 
     * @param direction Direction to run the motors in
     * @return Command that runs the dynamic test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.subsystems.Pivot.Constants.*;

/** Subsystem which rotates manipulator. */
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Position> {
    /**
     * Direction of motor rotation defined as positive rotation. Defined for
     * manipulator pivot to be rotation away from zero point.
     */
    public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive; // TODO
    /** Ratio of motor rotations to pivot rotations. */
    public static final double GEAR_RATIO = 12 / 1;
    /** Ratio of motor rotations to pivot rotations, in radians. */
    public static final double SENSOR_TO_MECHANISM = GEAR_RATIO / (2 * Math.PI);
    /** Current limit of manipulator motor. */
    public static final double CURRENT_LIMIT = 10; // TODO find good # amps\\

    /** Constant values of manipualtor.. */
    public static final class Constants {
        /** CAN information of manipulator motor. */
        public static final class CAN {
            /** CAN bus pivot motor is on. */
            public static final String BUS = "rio";
            /** CAN ID of pivot motor. */
            public static final int ID = 13;
        }

        /** Positions pivot can be in. */
        public static enum Position {
            ZERO(0),
            TEMP(0),
            SOFT_STOP(0.092);

            public final double value;

            private Position(final double value) {
                this.value = value;
            }
        }

        /**
         * Constants for feedforward control for moving to position setpoints.
         */
        public static final class Feedforward {
            /** Voltage required to overcome gravity. */
            public static final double kG = 0; // TODO find good value
            /** Voltage required to overcome motor's static friction. */
            public static final double kS = 0; // TODO find good value
            /** Voltage required to maintain constant velocity on motor. */
            public static final double kV = 0; // TODO find good value
            /** Voltage required to induce a given acceleration on motor. */
            public static final double kA = 0; // TODO find good value
        }

        /**
         * Constants for PID feedback control for error correction for moving to
         * position setpoints.
         */
        public static final class Feedback {
            /** Proportional term constant which drives error to zero proportionally. */
            public static final double kP = 0; // TODO find good value
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
        public static final class MotionMagic {
            /** Target cruise velocity along course of motion. */
            public static final double CRUISE_VELOCITY = 0; // TODO
            /** Target acceleration of beginning and end of course of motion. */
            public static final double ACCELERATION = 0; // TODO
            /** Target jerk along course of motion. */
            public static final double JERK = 0; // TODO
        }
    }

    /** Pivot motor */
    private final TalonFX motor = new TalonFX(CAN.ID, CAN.BUS);
    /** Pivot motor configuration object. */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    /** Pivot motor Motion Magic voltage request object. */
    private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentRequest = new MotionMagicTorqueCurrentFOC(0);

    /** Mutable measure for voltages applied during sysId testing */
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    /** Mutable measure for distances traveled during sysId testing (Meters) */
    private final MutAngle sysIdAngle = Radian.mutable(0);
    /** Mutable measure for velocity during SysId testing (Meters/Second) */
    private final MutAngularVelocity sysIdVelocity = RadiansPerSecond.mutable(0);
    /**
     * The sysIdRoutine object with default configuration and logging of voltage,
     * velocity, and distance
     */
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(1), null,
                    state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(voltage -> {
                setVoltage(voltage.magnitude());
            }, log -> {
                log.motor("Pivot")
                        .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .angularPosition(sysIdAngle.mut_replace(getPosition(), Radian))
                        .angularVelocity(sysIdVelocity.mut_replace(getVelocity(), RadiansPerSecond));
            }, this));

    /** Initialize pivot motor configurations. */
    public Pivot() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        // motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        motorConfigs.Slot0.kS = Feedforward.kS;
        motorConfigs.Slot0.kG = Feedforward.kG;
        motorConfigs.Slot0.kV = Feedforward.kV;
        motorConfigs.Slot0.kA = Feedforward.kA;
        motorConfigs.Slot0.kP = Feedback.kP;
        motorConfigs.Slot0.kI = Feedback.kI;
        motorConfigs.Slot0.kD = Feedback.kD;

        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = MotionMagic.CRUISE_VELOCITY;
        motorConfigs.MotionMagic.MotionMagicAcceleration = MotionMagic.ACCELERATION;
        motorConfigs.MotionMagic.MotionMagicJerk = MotionMagic.JERK;

        motor.getConfigurator().apply(motorConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the current position of the pivot mechanism.
     * 
     * @return the position of the mechanism, in radians.
     */
    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Resets the position of the pivot mechanism to a specific value.
     */
    @Override
    public void resetPosition() {
        motor.setPosition(Position.ZERO.value);
    }

    /**
     * Explicitly set the voltage of the pivot mechanism's motor.
     * 
     * @param voltage the voltage [-12, 12]
     */
    @Override
    public void setVoltage(final double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * Command to continuously have the pivot mechanism move to the currently set
     * goal.
     * 
     * @return the command
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            motor.setControl(motionMagicTorqueCurrentRequest.withPosition(motionMagicTorqueCurrentRequest.Position));
        }).withName("pivot.moveToCurrentGoalCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToCurrentGoalCommand'");
    }

    /**
     * Command that sets the current goal to the setpoint,
     * and cancels once the pivot mechanism has reached that goal.
     * 
     * @param goalPositionSupplier a supplier of an instance of the position enum
     * @return the command
     */
    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> motor
                        .setControl(motionMagicTorqueCurrentRequest.withPosition(goalPositionSupplier.get().value))),
                moveToCurrentGoalCommand()).withName("pivot.moveToPositionCommand");
    }

    /**
     * Command that sets the current goal position to the setpoint,
     * and cancels once the pivot mechanism has reached that goal.
     * 
     * @param goalPositionSupplier a supplier of a position to move to, in radians
     * @return the command
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> motor
                        .setControl(motionMagicTorqueCurrentRequest.withPosition(goalPositionSupplier.get()))),
                moveToCurrentGoalCommand()).withName("pivot.moveToArbitraryPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToArbitraryPositionCommand'");
    }

    /**
     * Command that sets the current goal position to the setpoint plus
     * the delta (the additional distance to move, relative to the current
     * position), and
     * cancels once the pivot mechanism has reached that goal.
     * 
     * @param delta a supplier of a delta to move, in radians
     * @return the command
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> motionMagicTorqueCurrentRequest.Position + delta.get())
                .withName("pivot.movePositionDeltaCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'movePositionDeltaCommand'");
    }

    /**
     * Command that sets the goal to the current position, and moves to
     * that goal until cancelled.
     * 
     * @return the command
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return movePositionDeltaCommand(() -> 0.0).withName("pivot.holdCurrentPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'holdCurrentPositionCommand'");
    }

    /**
     * Creates an instantaneous command that resets the position of the pivot
     * mechanism.
     * 
     * @return the command
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("pivot.resetPosition");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'resetPositionCommand'");
    }

    /**
     * Creates a command that manually sets the speed of the pivot mechanism.
     * 
     * @param speed the speed [-1,1]
     * @return the command
     */
    @Override
    public Command setOverridenSpeedCommand(final Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("pivot.setOverriddenSpeedCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'setOverridenSpeedCommand'");
    }

    /**
     * Command to stop the motor, sets it to coast mode, and then set it back to
     * brake mode on end. Allows for
     * moving the pivot mechanism manually.
     * 
     * @return the command
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor).andThen(() -> {
            motor.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo((d) -> {
            motor.setNeutralMode(NeutralModeValue.Coast);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("pivot.coastMotorsCommand");
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

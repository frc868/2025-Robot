package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.Constants.CAN;
import frc.robot.subsystems.Climber.Constants.Feedback;
import frc.robot.subsystems.Climber.Constants.Feedforward;
import frc.robot.subsystems.Climber.Constants.MotionProfile;
import frc.robot.subsystems.Climber.Constants.Position;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.subsystems.Climber.Constants.*;

/** Subsystem which hangs robot from deep cage. */
public class Climber extends SubsystemBase implements BaseSingleJointedArm<Position> {
    /** Constant values of climber subsystem. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * climber motors to be rotation away from zero point.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
        /** Current limit of climber motors. */
        public static final double CURRENT_LIMIT = 0; // TODO
        public static final double GEAR_RATIO = 36 / (0.75 * Math.PI); // TODO

        /** CAN information of climber motors. */
        public static final class CAN {
            /** CAN bus climber motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of climber motors. */
            public static final class IDs {
                /** CAN ID of climber motor A. */
                private static final int MOTOR_A = 9;
                /** CAN ID of climber motor B. */
                private static final int MOTOR_B = 10;
            }
        }

        /** Positions climber can be in. */
        public enum Position {
            ZERO(0.0), // TODO
            CLAMPED(0.0); // TODO

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
        public static final class MotionProfile {
            /** Target cruise velocity along course of motion. */
            public static final double CRUISE_VELOCITY = 0; // TODO
            /** Target acceleration of beginning and end of course of motion. */
            public static final double ACCELERATION = 0; // TODO
            /** Target jerk along course of motion. */
            public static final double JERK = 0; // TODO
        }
    }

    /** Climber motor A. */
    private final TalonFX motorA = new TalonFX(CAN.IDs.MOTOR_A, CAN.BUS);
    /** Climber motor B. */
    private final TalonFX motorB = new TalonFX(CAN.IDs.MOTOR_B, CAN.BUS);
    /**
     * Configuration object for configurations shared across both climber motors.
     */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(
            Position.ZERO.position);

    /** Mutable measure for voltages applied during sysId testing */
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    /** Mutable measure for distances traveled during sysId testing (Meters) */
    private final MutAngle sysIdAngle = Degrees.mutable(0);
    /** Mutable measure for velocity during SysId testing (Meters/Second) */
    private final MutAngularVelocity sysIdVelocity = DegreesPerSecond.mutable(0);
    /**
     * The sysIdRoutine object with default configuration and logging of voltage,
     * velocity, and distance
     */
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((voltage) -> {
                setVoltage(voltage.magnitude());
            }, (log) -> {
                log.motor("Climber")
                        .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .angularPosition(sysIdAngle.mut_replace(getPosition(), Degrees))
                        .angularVelocity(sysIdVelocity.mut_replace(getVelocity(), DegreesPerSecond));
            }, this));

    /** Initial climber motors configurations. */
    public Climber() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

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

        motorA.getConfigurator().apply(motorConfigs);
        motorB.getConfigurator().apply(motorConfigs);
    }

    /**
     * Gets the position of the left motor, increasing as it goes up
     * 
     * @return position of the left motor, as a double
     */
    @Override
    public double getPosition() {
        return motorA.getPosition(true).getValueAsDouble();
    }

    /**
     * Finds the voltage of the {@link #motorA motor}
     * 
     * @return Voltage of the motor, as a double
     */
    public double getVoltage() {
        return motorA.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Finds the velocity of the {@link #motorA motor}
     * 
     * @return velocity of the motor, as a double
     */
    public double getVelocity() {
        return motorA.getVelocity().getValueAsDouble();
    }

    /**
     * Resets the position of the motors to the reset position specified in
     * {@link Position#ZERO the constants}
     */
    @Override
    public void resetPosition() {
        motorA.setPosition(Position.ZERO.position);
        motorB.setPosition(Position.ZERO.position);
    }

    /**
     * Sets the voltage of both climber motors to a clamped value
     * 
     * @param voltage Voltage to set the motors to, clamped to the range [-12,12]
     */
    @Override
    public void setVoltage(double voltage) {
        motorA.setVoltage(MathUtil.clamp(voltage, -12, 12));
        motorB.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    /**
     * Moves the climber motors to their current goal, if not already doing so/
     * 
     * @return Command to move to the current PID goal
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return moveToArbitraryPositionCommand(() -> motionMagicVoltageRequest.Position)
                .withName("climber.moveToCurrentGoal");
    }

    /**
     * Sets the goal and moves the climber motors to the supplied position from
     * {@link Position the possible positions}
     * 
     * @return Command that will move to one of the possible preset positions
     */
    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
                .withName("climber.moveToPosition");
    }

    /**
     * Sets the goal and moves the climber motors to the supplied position.
     * 
     * @param goalPositionSupplier The supplier for the goal position, which must
     *                             return a double
     * @return Command that will move to a specified position
     */
    @Override
    public Command moveToArbitraryPositionCommand(final Supplier<Double> goalPositionSupplier) {
        return runOnce(() -> {
            motorA.setControl(motionMagicVoltageRequest.withPosition(goalPositionSupplier.get()));
            motorB.setControl(motionMagicVoltageRequest.withPosition(goalPositionSupplier.get()));
        }).withName("climber.moveToArbitraryPosition");
    }

    /**
     * Sets the goal and moves climber motors to a position relative to the current
     * position
     * 
     * @param delta Supplier for the relative change to the current position
     *              (Current position + delta)
     * @return Command that will move the climber relative to its current position
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> delta.get() + getPosition()).withName("climber.movePositionDelta");
    }

    /**
     * Holds the current position of the climber motors, keeping the climber
     * mechanism still.
     * 
     * @return Command that will keep the climber still
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return moveToArbitraryPositionCommand(() -> getPosition()).withName("climber.holdCurrentPosition");
    }

    /**
     * Resets the position of the climber encoders.
     * This is a Command based wrapper for {@link #resetPosition()}
     * 
     * @return Command that resets the position to
     *         {@link Position#ZERO the reset position}
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("climber.resetPosition");
    }

    /**
     * Creates a command that overrides PID control for the climber.
     * 
     * @param speed Supplier for speed, which should return values between -1 to 1
     * @return Command that will set the speed of climber until interrupted, and
     *         then zero the voltage.
     */
    @Override
    public Command setOverridenSpeedCommand(final Supplier<Double> speed) {
        return runEnd(() -> setVoltage(speed.get() * 12.0), () -> setVoltage(0)).withName("climber.setOverridenSpeed");
    }

    /**
     * Creates a command that will make the climber motors coast instead of brake.
     * Once done, climber motors will go back to braking.
     * 
     * @return Command that causes motors to coast
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            motorA.stopMotor();
            motorB.stopMotor();
        }).andThen(() -> {
            motorA.setNeutralMode(NeutralModeValue.Coast);
            motorB.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            motorA.setNeutralMode(NeutralModeValue.Brake);
            motorB.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("climber.coastMotors");
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

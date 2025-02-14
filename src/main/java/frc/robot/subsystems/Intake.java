package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Intake.Constants.Pivot;
import static frc.robot.subsystems.Intake.Constants.*;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

/** Subsystem which intakes algae from ground. */
public class Intake extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Pivot.Position> {
    /** Constant values of intake subsystem. */
    public static final class Constants {
        /** CAN information of intake motors. */
        public static final class CAN {
            /** CAN bus intake motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of intake motors. */
            public static final class IDs {
                /** CAN ID of intake pivot left motor. */
                private static final int PIVOT_LEFT = 17;
                /** CAN ID of intake pivot right motor. */
                private static final int PIVOT_RIGHT = 16;
                /** CAN ID of intake rollers motor. */
                private static final int ROLLERS = 15;
            }
        }

        /** Constant values of intake pivot. */
        public static final class Pivot {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * pivot to be rotation away from zero point.
             */
            public static final InvertedValue LEFT_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
            /** Ratio of motor rotations to intake pivot rotations. */
            public static final double SENSOR_TO_MECHANISM = 24.0 / 11.0;
            /** Intake pivot motor current limit. */
            public static final double CURRENT_LIMIT = 0; // TODO

            /** Positions intake pivot can be in. */
            public static enum Position {
                ZERO(0), // TODO
                GROUND_INTAKE(0), // TODO
                MANIPULATOR_INTAKE(0); // TODO

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
            public static final class MotionMagic {
                /** Target cruise velocity along course of motion. */
                public static final double CRUISE_VELOCITY = 0; // TODO
                /** Target acceleration of beginning and end of course of motion. */
                public static final double ACCELERATION = 0; // TODO
                /** Target jerk along course of motion. */
                public static final double JERK = 0; // TODO
            }
        }

        /** Constant values of intake rollers. */
        public static final class Rollers {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * rollers to be rotation which intakes algae.
             */
            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TOD
            /** Intake rollers motor current limit. */
            public static final double CURRENT_LIMIT = 0; // TODO
            /** Voltage to run intake rollers motor at. */
            public static final double VOLTAGE = 0; // TODO
        }
    }

    /** Intake pivot left motor. */
    private final TalonFX pivotLeftMotor = new TalonFX(CAN.IDs.PIVOT_LEFT, CAN.BUS);
    /** Intake pivot right motor. */
    private final TalonFX pivotRightMotor = new TalonFX(CAN.IDs.PIVOT_RIGHT, CAN.BUS);
    /** Intake pivot motor configuration object. */
    private final TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage pivotMotionMagicVoltageRequest = new MotionMagicVoltage(
            Pivot.Position.ZERO.position);
    private final VoltageOut directVoltageRequest = new VoltageOut(0);

    /** Intake rollers motor. */
    private final TalonFX rollersMotor = new TalonFX(CAN.IDs.ROLLERS, CAN.BUS);
    /** Intake rollers motor configuration object. */
    private final TalonFXConfiguration rollersMotorConfigs = new TalonFXConfiguration();
    /** Intake rollers motor voltage request object. */
    private final VoltageOut rollersVoltageRequest = new VoltageOut(0);

    /** SysID weee */
    private final MutVoltage sysIdVoltage = Volts.mutable(0);
    private final MutAngle sysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity sysIdVelocity = DegreesPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((voltage) -> {
                setVoltage(voltage.magnitude());
            }, (log) -> {
                log.motor("Intake")
                        .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                        .angularPosition(sysIdAngle.mut_replace(getPosition(), Degrees))
                        .angularVelocity(sysIdVelocity.mut_replace(getVelocity(), DegreesPerSecond));
            }, this));

    /** Initialize intake pivot and rollers motor configurations. */
    public Intake() {
        pivotMotorConfigs.MotorOutput.Inverted = Pivot.LEFT_MOTOR_DIRECTION;

        pivotMotorConfigs.Feedback.SensorToMechanismRatio = Pivot.SENSOR_TO_MECHANISM;

        pivotMotorConfigs.CurrentLimits.SupplyCurrentLimit = Pivot.CURRENT_LIMIT;

        pivotMotorConfigs.Slot0.kG = Pivot.Feedforward.kG;
        pivotMotorConfigs.Slot0.kS = Pivot.Feedforward.kS;
        pivotMotorConfigs.Slot0.kV = Pivot.Feedforward.kV;
        pivotMotorConfigs.Slot0.kA = Pivot.Feedforward.kA;
        pivotMotorConfigs.Slot0.kP = Pivot.Feedback.kP;
        pivotMotorConfigs.Slot0.kI = Pivot.Feedback.kI;
        pivotMotorConfigs.Slot0.kD = Pivot.Feedback.kD;

        pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Pivot.MotionMagic.CRUISE_VELOCITY;
        pivotMotorConfigs.MotionMagic.MotionMagicAcceleration = Pivot.MotionMagic.ACCELERATION;
        pivotMotorConfigs.MotionMagic.MotionMagicJerk = Pivot.MotionMagic.JERK;

        pivotLeftMotor.getConfigurator().apply(pivotMotorConfigs);
        pivotRightMotor.getConfigurator().apply(pivotMotorConfigs);

        pivotRightMotor.setControl(new Follower(pivotLeftMotor.getDeviceID(), true));

        rollersMotorConfigs.MotorOutput.Inverted = Rollers.MOTOR_DIRECTION;

        rollersMotorConfigs.CurrentLimits.SupplyCurrentLimit = Rollers.CURRENT_LIMIT;

        rollersMotor.getConfigurator().apply(rollersMotorConfigs);
    }

    @Override
    public double getPosition() {
        return pivotLeftMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return pivotLeftMotor.getVelocity().getValueAsDouble();
    }

    public double getVoltage() {
        return pivotLeftMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        pivotLeftMotor.setPosition(Pivot.Position.ZERO.position);
        pivotRightMotor.setPosition(Pivot.Position.ZERO.position);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotLeftMotor.setControl(directVoltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            pivotLeftMotor
                    .setControl(pivotMotionMagicVoltageRequest.withPosition(pivotMotionMagicVoltageRequest.Position));
        }).withName("intake.moveToCurrentGoalCommand");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Pivot.Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
                .withName("intake.moveToPositionCommand");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    pivotLeftMotor.setControl(pivotMotionMagicVoltageRequest.withPosition(goalPositionSupplier.get()));
                }), moveToCurrentGoalCommand()).withName("intake.moveToArbitraryPositionCommand");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pivotMotionMagicVoltageRequest.Position + delta.get())
                .withName("intake.movePositionDeltaCommand");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return movePositionDeltaCommand(() -> 0.0)
                .withName("intake.holdCurrentPositionCommand");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).ignoringDisable(true)
                .withName("intake.resetPositionCommand");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("intake.setOverridenSpeedCommand");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            pivotLeftMotor.stopMotor();
        }).andThen(() -> {
            pivotLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo((s) -> {
            pivotLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("intake.coastMotorsCommand");
    }

    @Override
    public Command runRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.runRollersCommand");
    }

    @Override
    public Command reverseRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(-Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.reverseRollersCommand");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
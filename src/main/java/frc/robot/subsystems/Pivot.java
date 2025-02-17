package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Pivot.Constants.CAN;
import frc.robot.subsystems.Pivot.Constants.Feedback;
import frc.robot.subsystems.Pivot.Constants.Feedforward;
import frc.robot.subsystems.Pivot.Constants.MotionMagic;
import frc.robot.subsystems.Pivot.Constants.Position;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

/** Subsystem which rotates manipulator. */
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Position> {
    // public static TunableDouble p = new TunableDouble("subsystems/pivot/kP", 0);
    /**
     * Direction of motor rotation defined as positive rotation. Defined for
     * manipulator pivot to be rotation away from zero point.
     */
    public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
    /** Ratio of motor rotations to pivot rotations. */
    public static final double GEAR_RATIO = 12 / 1;
    /** Ratio of motor rotations to pivot rotations, in radians. */
    public static final double SENSOR_TO_MECHANISM = GEAR_RATIO;
    /** Current limit of manipulator motor. */
    public static final double CURRENT_LIMIT = 60; // TODO find good # amps\\

    public static double p = 0;

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
            HARD_STOP(0.405029296875),
            PAST_ELEVATOR(0.04922),
            L1(0),
            L2(-0.06),
            L3(-0.06),
            L4(-0.06),
            SOFT_STOP(0.092);

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
            public static final double kG = 0.62; // TODO find good value
            /** Voltage required to overcome motor's static friction. */
            public static final double kS = 0.832; // TODO find good value
            /** Voltage required to maintain constant velocity on motor. */
            public static final double kV = 1.49; // TODO find good value
            /** Voltage required to induce a given acceleration on motor. */
            public static final double kA = 0.13; // TODO find good value
        }

        /**
         * Constants for PID feedback control for error correction for moving to
         * position setpoints.
         */
        public static final class Feedback {
            /** Proportional term constant which drives error to zero proportionally. */
            public static double kP = 25; // TODO find good value
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
            public static final double CRUISE_VELOCITY = 2; // TODO
            /** Target acceleration of beginning and end of course of motion. */
            public static final double ACCELERATION = 2; // TODO
            /** Target jerk along course of motion. */
            public static final double JERK = 0; // TODO
        }
    }

    /** Pivot motor */
    private final TalonFX motor = new TalonFX(CAN.ID, CAN.BUS);
    /** Pivot motor configuration object. */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(Position.HARD_STOP.position);
    /** Request object for setting elevator motors' voltage directly. */
    private final VoltageOut voltageRequest = new VoltageOut(0);
    /** Request object for stopping the motor. */
    private final NeutralOut stop = new NeutralOut();

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
    private SysIdRoutine sysIdRoutine;

    /**
     * Global position tracker object for tracking mechanism positions for
     * calculating safeties.
     */
    private PositionTracker positionTracker;

    private boolean initalized = false;

    /** Initialize pivot motor configurations. */
    public Pivot(PositionTracker positionTracker) {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

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

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.125).per(Second), Volts.of(0.5), null,
                        state -> {
                            SignalLogger.writeString("state", state.toString());
                            sysIdRoutine.recordState(state);
                        }),
                new SysIdRoutine.Mechanism(voltage -> {
                    setVoltage(voltage.magnitude());
                }, log -> {
                    log.motor("Pivot")
                            .voltage(sysIdVoltage.mut_replace(getVoltage(), Volts))
                            .angularPosition(sysIdAngle.mut_replace(getPosition(), Rotation))
                            .angularVelocity(sysIdVelocity.mut_replace(getVelocity(), RotationsPerSecond));
                }, this));

        this.positionTracker = positionTracker;
        positionTracker.addPositionSupplier("Pivot", this::getPosition);

        setDefaultCommand(holdCurrentPositionCommand());
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public ControlRequest outputRequestWithSafeties(ControlRequest controlRequest) {
        if (!initalized) {
            return stop;
        }

        if (positionTracker.getPosition("Elevator") > 0) {
            return motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position).withEnableFOC(true);
        }

        return controlRequest;
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        motor.setPosition(Position.HARD_STOP.position);
    }

    @Override
    public void setVoltage(final double voltage) {
        motor.setControl(outputRequestWithSafeties(voltageRequest.withOutput(voltage)));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> motor.setControl(outputRequestWithSafeties(
                motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position).withEnableFOC(true))));
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
                .withName("pivot.moveToPositionCommand");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> motor
                        .setControl(outputRequestWithSafeties(motionMagicVoltageRequest
                                .withPosition(goalPositionSupplier.get()).withEnableFOC(true)))),
                moveToCurrentGoalCommand()).withName("pivot.moveToArbitraryPositionCommand");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> motionMagicVoltageRequest.Position + delta.get())
                .withName("pivot.movePositionDeltaCommand");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(
                () -> motor.setControl(outputRequestWithSafeties(
                        motionMagicVoltageRequest.withPosition(getPosition()).withEnableFOC(true))))
                .withName("pivot.holdCurrentPositionCommand");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> {
            resetPosition();
            initalized = true;
        }).withName("pivot.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(final Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("pivot.setOverriddenSpeedCommand");
    }

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

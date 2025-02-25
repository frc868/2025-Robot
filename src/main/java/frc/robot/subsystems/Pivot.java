package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Pivot.Constants.*;

/** Subsystem which rotates manipulator. */
@LoggedObject
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Position> {
    /** Constant values of manipulator pivot. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * manipulator pivot to be rotation which raises manipulator when moving away
         * from the pivot where the force of gravity is the strongest on the manipulator
         * and manipuator pivot.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
        /** Ratio of motor rotations to pivot rotations. */
        public static final double SENSOR_TO_MECHANISM = 12 / 1;
        /** Current limit of manipulator motor. */
        public static final double CURRENT_LIMIT = 60;
        public static final double POSITION_TOLERANCE = 0.01;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(1);
        public static final double GEARING = 12.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(5); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(-48);
        public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(170);

        /** CAN information of manipulator pivot. */
        public static final class CAN {
            /** CAN bus manipulator pivot motor is on. */
            public static final String BUS = "rio";
            /** CAN ID of manipulator pivot motor. */
            public static final int ID = 13;
        }

        /**
         * Positions manipulator pivot can be in, in manipulator pivot rotations. 0 is
         * defined to be where the force of gravity is the
         */
        public static enum Position {
            HARD_STOP(0.405029296875),
            GROUND_ALGAE(-0.19), // TODO
            PROCESSOR(0), // TODO
            NET(0.13),
            PAST_ELEVATOR(0.07),
            SAFE(0),
            L1(0.03),
            L2(-0.06),
            L3(-0.06),
            L4(-0.07),
            ALGAE(-0.01),
            SOFT_STOP(0.092),
            CLIMB_RETRACT(0.2);

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
    // @Log
    private final TalonFX motor = new TalonFX(CAN.ID, CAN.BUS);
    /** Configuration object for pivot motor. */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    /**
     * Request object for motor voltage according to Motion Magic motion profile.
     */
    private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(Position.HARD_STOP.position);
    /** Request object for setting elevator motors' voltage directly. */
    private final VoltageOut voltageRequest = new VoltageOut(0);
    /** Request object for stopping the motor. */
    private final NeutralOut stopRequest = new NeutralOut();

    /**
     * SysId routine to run to empirically determine feedforward and feedback
     * constant values, using CTRE's Phoenix 6 {@link SignalLogger Signal Logger}.
     */
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.125).per(Second), Volts.of(0.5), null,
                    state -> {
                        SignalLogger.writeString("state", state.toString());
                    }),
            new SysIdRoutine.Mechanism(voltage -> {
                setVoltage(voltage.magnitude());
            }, null, this));

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOI,
            COM_DISTANCE_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            Units.rotationsToRadians(Position.HARD_STOP.position));

    /**
     * Global position tracker object for tracking mechanism positions for
     * calculating safeties.
     */
    private PositionTracker positionTracker;

    /**
     * Whether manipulator pivot position has been reset while the manipulator pivot
     * is resting against its hard stop using. If uninitialized, the mechanism
     * should not be able to move at all. Initialized to {@code false} until
     * position is reset by {@link frc.robot.HoundBrian HoundBrian}.
     */
    @Log
    private boolean initalized = false;

    /**
     * Initialize manipulator pivot configurations and add pivot position to global
     * position tracker.
     * 
     * @param positionTracker global position tracker object
     */
    public Pivot(PositionTracker positionTracker) {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        // motorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

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

        this.positionTracker = positionTracker;
        positionTracker.addPositionSupplier("Pivot", this::getPosition);

        // setDefaultCommand(holdCurrentPositionCommand());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = motor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(0.020);

        talonFXSim.setRawRotorPosition(armSim.getAngleRads() / (2 * Math.PI) * GEARING);
        talonFXSim.setRotorVelocity(armSim.getVelocityRadPerSec() / (2 * Math.PI) * GEARING);

    }

    @Log(groups = "components")
    public Pose3d getComponentPose() {
        // since 0 position needs to be horizontal
        return new Pose3d(0.275, 0, 0.435 + positionTracker.getPosition("Elevator"), new Rotation3d(0, -0.45, 0))
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, -getPosition(), 0)));
    }

    public Transform3d getCoralTransform() {
        // since 0 position needs to be horizontal
        return new Transform3d(new Pose3d(),
                getComponentPose()
                        .plus(new Transform3d(0.3, 0, -0.1325, new Rotation3d(0, Units.degreesToRadians(-130), 0))));
    }

    public Transform3d getAlgaeTransform() {
        // since 0 position needs to be horizontal
        return new Transform3d(new Pose3d(),
                getComponentPose()
                        .plus(new Transform3d(0.42, 0, -0.3, new Rotation3d())));
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
    public ControlRequest outputRequestWithSafeties(ControlRequest controlRequest) {
        if (!initalized) {
            return stopRequest;
        }

        // if (positionTracker.getPosition("Elevator") > 0) {
        // return
        // motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position).withEnableFOC(true);
        // }

        return controlRequest;
    }

    @Log
    public boolean atGoal() {
        return Math.abs(getPosition() - motionMagicVoltageRequest.Position) < POSITION_TOLERANCE;
    }

    // @Override%
    // public double getPosition() {
    // return motor.getPosition().getValue().in(Radians);
    // }

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
                motionMagicVoltageRequest.withPosition(motionMagicVoltageRequest.Position).withEnableFOC(true))))
                .until(this::atGoal).withName("pivot.moveToCurrentGoalCommand");
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
        return runOnce(() -> {
            double currentPosition = getPosition();

            if (currentPosition >= Position.HARD_STOP.position) {
                motor.setControl(stopRequest);
            } else {
                motor.setControl(outputRequestWithSafeties(
                        motionMagicVoltageRequest.withPosition(currentPosition).withEnableFOC(true)));
            }
        }).withName("pivot.holdCurrentPositionCommand");
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

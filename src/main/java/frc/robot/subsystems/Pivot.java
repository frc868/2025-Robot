package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** Subsystem which rotates manipulator subsystem. */
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Pivot.Constants.Position> {
    /** Constant values of pivot subsystem. */
    public static final class Constants {
        private static final class CANIDs {
            private static final int pivotMotor = 0; // TODO get actual can ids
        }

        public static class PID {
            public static final double kP = 0; // TODO find good value
            public static final double kI = 0; // TODO find good value
            public static final double kD = 0; // TODO find good value
        }

        public static class FeedForward {
            public static final double kS = 0; // TODO find good value
            public static final double kG = 0; // TODO find good value
            public static final double kV = 0; // TODO find good value
            public static final double kA = 0; // TODO find good value
        }

        public static class MotionProfiling { // TODO everything
            public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.0;
            public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.0;
            public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                    MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

        }

        public static final int MAX_AMPS = 10;

        /** Positions that pivot subsystem can be in. */
        public enum Position {
            // TODO measure positions
        }
    }

    private SparkFlex motor;
    private SparkFlexConfig motorConfig;
    private ProfiledPIDController pidController = new ProfiledPIDController(Constants.PID.kP, Constants.PID.kI,
            Constants.PID.kD,
            Constants.MotionProfiling.MOVEMENT_CONSTRAINTS);
    private ArmFeedforward feedforward = new ArmFeedforward(Constants.FeedForward.kS, Constants.FeedForward.kG,
            Constants.FeedForward.kV, Constants.FeedForward.kA);
    private double feedbackVoltage = 0.0;
    private double feedforwardVoltage = 0.0;

    public Pivot() {
        // TODO everything
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
        // throw new UnsupportedOperationException("Unimplemented method
        // 'getPosition'");
    }

    @Override
    public void resetPosition() {
        motor.getEncoder().setPosition(-65537); // TODO real value
        // throw new UnsupportedOperationException("Unimplemented method
        // 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // TODO safety stuff
        motor.setVoltage(voltage);
        // throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforward.calculate(getPosition(), 0); // TODO is velocity really 0?
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("pivot.moveToCurrentGoal");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Constants.Position> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)), // TODO this is some enum stuff
                moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToPosition");

        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToPositionCommand'");
    }

    /* this was in the code last year so good enough */
    public boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())), // TODO this is some enum stuff
                moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToArbitraryPosition");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("pivot.movePositionDeltaCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return movePositionDeltaCommand(() -> 0.0).withName("pivot.holdCurrentPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("pivot.resetPosition");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("pivot.setOverriddenSpeed");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> motorConfig.idleMode(IdleMode.kCoast)) // TODO make sure im doing this right
                .finallyDo((d) -> {
                    motorConfig.idleMode(IdleMode.kBrake); // TODO make sure im doing this right
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("pivot.coastMotorsCommand");

        // throw new UnsupportedOperationException("Unimplemented method
        // 'coastMotorsCommand'");
    }
}

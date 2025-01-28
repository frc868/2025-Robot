package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

/** Subsystem which rotates pivot subsystem. */
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Pivot.Constants.Position> {
    /** Constant values of pivot subsystem. */
    public static final class Constants {
        private static final class CANIDs {
            private static final int PIVOT_MOTOR = 0; // TODO get actual can ids
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
        public static final double RESET_POSITION = 0; //TODO get real reset encoder value

        /** Positions that pivot subsystem can be in. */
        public enum Position {
            TEMP(0);

            public final double value;

            private Position(double value) {
                this.value = value;
            }
        }

    }

    private TalonFX pivotMotor = new TalonFX(Constants.CANIDs.PIVOT_MOTOR);
    private TalonFXConfigurator pivotConfigurator = pivotMotor.getConfigurator();
    private TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private CurrentLimitsConfigs pivotConfig_Current = new CurrentLimitsConfigs();

    public Pivot() {
        pivotConfig_Current.SupplyCurrentLimit = Constants.MAX_AMPS;
        pivotConfig_Current.SupplyCurrentLimitEnable = true;
        pivotConfigurator.apply(pivotConfig_Current);
    }

    private ProfiledPIDController pidController = new ProfiledPIDController(Constants.PID.kP, Constants.PID.kI, Constants.PID.kD, Constants.MotionProfiling.MOVEMENT_CONSTRAINTS);
    private ArmFeedforward feedforward = new ArmFeedforward(Constants.FeedForward.kS, Constants.FeedForward.kG, Constants.FeedForward.kV, Constants.FeedForward.kA);
    private double feedbackVoltage = 0.0;
    private double feedforwardVoltage = 0.0;

    /**
     * Gets the current position of the pivot mechanism.
     * 
     * @return the position of the mechanism, in radians.
     */
    @Override
    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
        // throw new UnsupportedOperationException("Unimplemented method
        // 'getPosition'");
    }

    /**
     * Resets the position of the pivot mechanism to a specific value.
     */
    @Override
    public void resetPosition() {
        pivotMotor.setPosition(Constants.RESET_POSITION);
        // throw new UnsupportedOperationException("Unimplemented method
        // 'resetPosition'");
    }

    /**
     * Explicitly set the voltage of the pivot mechanism's motor.
     * 
     * @param voltage the voltage [-12, 12]
     */
    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // TODO safety stuff
        pivotMotor.setVoltage(voltage);
        // throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    /**
     * Command to continuously have the pivot mechanism move to the currently set goal.
     * 
     * @return the command
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            feedforwardVoltage = feedforward.calculate(getPosition(), 0); // TODO is velocity really 0?
            setVoltage(feedbackVoltage + feedforwardVoltage);
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
    public Command moveToPositionCommand(Supplier<Constants.Position> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> pidController.reset(getPosition())),
            runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
            moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToPositionCommand");

        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToPositionCommand'");
    }

    /* this was in the code last year so good enough */
    public boolean atGoal() {
        return pidController.atGoal();
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
            runOnce(() -> pidController.reset(getPosition())),
            runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
            moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToArbitraryPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'moveToArbitraryPositionCommand'");
    }

    /**
     * Command that sets the current goal position to the setpoint plus
     * the delta (the additional distance to move, relative to the current position), and
     * cancels once the pivot mechanism has reached that goal.
     * 
     * @param delta a supplier of a delta to move, in radians
     * @return the command
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
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
     * Creates an instantaneous command that resets the position of the pivot mechanism.
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
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
            .withName("pivot.setOverriddenSpeedCommand");
        // throw new UnsupportedOperationException("Unimplemented method
        // 'setOverridenSpeedCommand'");
    }

    /**
     * Command to stop the motor, sets it to coast mode, and then set it back to brake mode on end. Allows for
     * moving the pivot mechanism manually.
     * 
     * @return the command
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(pivotMotor::stopMotor)
            .andThen(() -> {
                pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                pivotConfigurator.apply(pivotConfiguration.MotorOutput); // TODO make sure this is right (when we
                                                                         // know stuff better)
            }).finallyDo((d) -> {
                pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                pivotConfigurator.apply(pivotConfiguration.MotorOutput);
                pidController.reset(getPosition());
            }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("pivot.coastMotorsCommand");

        // throw new UnsupportedOperationException("Unimplemented method
        // 'coastMotorsCommand'");
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
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

        public static final class PID {
            public static final double kP = 0; // TODO find good value
            public static final double kI = 0; // TODO find good value
            public static final double kD = 0; // TODO find good value
        }

        public static final class FeedForward {
            public static final double kS = 0; // TODO find good value
            public static final double kG = 0; // TODO find good value
            public static final double kV = 0; // TODO find good value
            public static final double kA = 0; // TODO find good value
            public static final double MM_ACCEL = 0; // TODO find good value
            public static final double MM_CRUISE = 0; // TODO find good value
            public static final double MM_JERK = 0; // TODO find good value
        }

        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO Clockwise_Positive or CounterClockwise_Positive
        
        public static final double MAX_AMPS = 10; // TODO find good # amps

        public static final double ENCODER_CONVERSION_FACTOR = 0 * 2 * Math.PI; // TODO get conversion factor for encoder rotations -> radians (replace 0 with gear ratio)

        /** Positions that pivot subsystem can be in. */
        public static enum Position {
            TEMP(0),
            RESET_POSITION(0);

            public final double value;

            private Position(double value) {
                this.value = value;
            }
        }
    }

    // Make motor object
    private final TalonFX pivotMotor = new TalonFX(Constants.CANIDs.PIVOT_MOTOR);

    // Make configs
    private final TalonFXConfigurator pivotConfigurator = pivotMotor.getConfigurator();
    private final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private final CurrentLimitsConfigs pivotConfigCurrent = new CurrentLimitsConfigs();
    private final FeedbackConfigs pivotConfigFeedback = new FeedbackConfigs();
    private final Slot0Configs pivotConfigControl = new Slot0Configs();
    private final MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    private final MotorOutputConfigs pivotConfigOutput = new MotorOutputConfigs();

    // Make motion magic request object
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // Constructor
    public Pivot() {
        // Apply configs
        pivotConfigCurrent.SupplyCurrentLimit = Constants.MAX_AMPS;
        pivotConfigCurrent.SupplyCurrentLimitEnable = true;
        pivotConfigurator.apply(pivotConfigCurrent);

        pivotConfigControl.kP = Constants.PID.kP;
        pivotConfigControl.kI = Constants.PID.kI;
        pivotConfigControl.kD = Constants.PID.kD;
        pivotConfigControl.kS = Constants.FeedForward.kS;
        pivotConfigControl.kG = Constants.FeedForward.kG;
        pivotConfigControl.kV = Constants.FeedForward.kV;
        pivotConfigControl.kA = Constants.FeedForward.kA;
        pivotConfigurator.apply(pivotConfigControl);

        pivotConfigFeedback.RotorToSensorRatio = Constants.ENCODER_CONVERSION_FACTOR;
        pivotConfigurator.apply(pivotConfigFeedback);

        pivotConfigOutput.Inverted = Constants.MOTOR_DIRECTION;
        pivotConfigurator.apply(pivotConfigOutput);

        mmConfig.MotionMagicAcceleration = Constants.FeedForward.MM_ACCEL;
        mmConfig.MotionMagicCruiseVelocity = Constants.FeedForward.MM_CRUISE;
        mmConfig.MotionMagicJerk = Constants.FeedForward.MM_JERK;
        pivotConfigurator.apply(mmConfig);
    }

    /**
     * Gets the current position of the pivot mechanism.
     * 
     * @return the position of the mechanism, in radians.
     */
    @Override
    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
        // throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    /**
     * Resets the position of the pivot mechanism to a specific value.
     */
    @Override
    public void resetPosition() {
        pivotMotor.setPosition(Constants.Position.RESET_POSITION.value);
        // throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
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
            pivotMotor.setControl(mmRequest.withPosition(mmRequest.Position));
        }).withName("pivot.moveToCurrentGoalCommand");
        // throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
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
            runOnce(() -> pivotMotor.setControl(mmRequest.withPosition(goalPositionSupplier.get().value))),
            moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    /* couldn't find a method to see if motion magic has reached its goal, so... if statement, yay.
    I doubt this works perfectly due to allowed tolerances in position probably being needed, but ¯\_(ツ)_/¯ */
    public boolean atGoal() {
        if (mmRequest.Position == pivotMotor.getPosition().getValueAsDouble()) {
            return true;
        } else {
            return false;
        }
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
            runOnce(() -> pivotMotor.setControl(mmRequest.withPosition(goalPositionSupplier.get()))),
            moveToCurrentGoalCommand().until(this::atGoal)).withName("pivot.moveToArbitraryPositionCommand");
        // throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
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
        return moveToArbitraryPositionCommand(() -> mmRequest.Position + delta.get())
            .withName("pivot.movePositionDeltaCommand");
        // throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
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
        // throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    /**
     * Creates an instantaneous command that resets the position of the pivot mechanism.
     * 
     * @return the command
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("pivot.resetPosition");
        // throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
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
        // throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
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
            }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("pivot.coastMotorsCommand");

        // throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}

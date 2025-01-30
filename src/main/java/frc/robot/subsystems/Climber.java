package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.Supplier;

/** Climber subsystem which hangs robot from deep cage. */
public class Climber extends SubsystemBase implements BaseSingleJointedArm<Climber.Constants.Position> {
    /** Constant values of climber subsystem. */
    public static final class Constants {
        // Declares motor CanIDs
        public static final class CANIDS {
            public static final int CLIMBER_MOTOR_LEFT_CANID = 0; // Have not asked what the CanID should be yet
            public static final int CLIMBER_MOTOR_RIGHT_CANID = 0; // Also have not asked what the CanID should be yet
        }

        public static class PID {
            public static final double kP = 0; // TODO find good value
            public static final double kI = 0; // TODO find good value
            public static final double kD = 0; // TODO find good value
        }

        public static class Feedforward {
            public static final double kS = 0; // TODO find good value
            public static final double kV = 0; // TODO find good value
            public static final double kA = 0; // TODO find good value
            public static final double MM_ACCEL = 0; // TODO find good value
            public static final double MM_CRUISE = 0; // TODO find good value
            public static final double MM_JERK = 0; // TODO find good value
        }

        public static final double CURRENT_LIMIT = 0; // Max current limit for climber
        // "Torque - we are not sure if we need it yet" - Sage Ryker and Noor(but
        // written by Sage Ryker)

        public static final double GEAR_RATIO = 36 / (.75 * Math.PI);

        public static final double VOLTAGE = 0;

        public static final InvertedValue MOTOR_L_INVERTED = InvertedValue.Clockwise_Positive;
        public static final InvertedValue MOTOR_R_INVERTED = InvertedValue.Clockwise_Positive;

        // Assign both climber motors to their specified CANID
        private static TalonFX climberMotorLeft = new TalonFX(Constants.CANIDS.CLIMBER_MOTOR_LEFT_CANID);
        private static TalonFX climberMotorRight = new TalonFX(Constants.CANIDS.CLIMBER_MOTOR_RIGHT_CANID);

        public enum Position {
            RESET_POSITION(0.0),
            CLAMPED(0.0);

            public final double pos;

            Position(double pos) {
                this.pos = pos;
            }
        }

    }

    // Use these to apply the configs
    private TalonFXConfigurator climberConfiguratorL = Constants.climberMotorLeft.getConfigurator();
    private TalonFXConfigurator climberConfiguratorR = Constants.climberMotorRight.getConfigurator();
    // Current limits
    private CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    // Convert rotations to usable output
    private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    private MotorOutputConfigs outputConfigsL = new MotorOutputConfigs();
    private MotorOutputConfigs outputConfigsR = new MotorOutputConfigs();
    private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private Slot0Configs controlConfigs = new Slot0Configs();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(Constants.Position.RESET_POSITION.pos);

    public Climber() {
        // Create current limits
        limitConfigs.SupplyCurrentLimit = Constants.CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLimitEnable = true;
        // Apply current limits
        climberConfiguratorL.apply(limitConfigs);
        climberConfiguratorR.apply(limitConfigs);

        // Make encoder output in terms of angular units for the climber's position
        feedbackConfigs.SensorToMechanismRatio = Constants.GEAR_RATIO;
        climberConfiguratorL.apply(feedbackConfigs);
        climberConfiguratorR.apply(feedbackConfigs);

        // Set motor inversions
        outputConfigsL.Inverted = Constants.MOTOR_L_INVERTED;
        outputConfigsR.Inverted = Constants.MOTOR_R_INVERTED;
        // Apply motor inversions
        climberConfiguratorL.apply(outputConfigsL);
        climberConfiguratorR.apply(outputConfigsR);

        // Set PID constants
        controlConfigs.kP = Constants.PID.kP;
        controlConfigs.kI = Constants.PID.kI;
        controlConfigs.kD = Constants.PID.kD;

        // Set feedforward constants
        controlConfigs.kA = Constants.Feedforward.kA;
        controlConfigs.kS = Constants.Feedforward.kS;
        controlConfigs.kV = Constants.Feedforward.kV;

        // Set motion magic configurations
        motionMagicConfigs.MotionMagicAcceleration = Constants.Feedforward.MM_ACCEL;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Feedforward.MM_CRUISE;
        motionMagicConfigs.MotionMagicJerk = Constants.Feedforward.MM_JERK;

        // Apply feedforward, PID, and motion magic configs
        climberConfiguratorL.apply(controlConfigs);
        climberConfiguratorR.apply(controlConfigs);

        climberConfiguratorL.apply(motionMagicConfigs);
        climberConfiguratorR.apply(motionMagicConfigs);

    }

    /**
     * Gets the position of the left motor, increasing as it goes up
     * 
     * @return position of the left motor, as a double
     */
    @Override
    public double getPosition() {
        return Constants.climberMotorLeft.getPosition(true).getValueAsDouble();
    }

    /**
     * Resets the position of the motors to the reset position specified in
     * {@link Constants.Position#RESET_POSITION the constants}
     */
    @Override
    public void resetPosition() {
        Constants.climberMotorLeft.setPosition(Constants.Position.RESET_POSITION.pos);
        Constants.climberMotorRight.setPosition(Constants.Position.RESET_POSITION.pos);
    }

    /**
     * Sets the voltage of both climber motors to a clamped value
     * 
     * @param voltage Voltage to set the motors to, clamped to the range [-12,12]
     */
    @Override
    public void setVoltage(double voltage) {
        Constants.climberMotorLeft.setVoltage(MathUtil.clamp(voltage, -12, 12));
        Constants.climberMotorRight.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    /**
     * Moves the climber motors to their current goal, if not already doing so/
     * 
     * @return Command to move to the current PID goal
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return moveToArbitraryPositionCommand(() -> mmRequest.Position).withName("climber.moveToCurrentGoal");
    }

    /**
     * Sets the goal and moves the climber motors to the supplied position from
     * {@link Constants.Position the possible positions}
     * 
     * @return Command that will move to one of the possible preset positions
     */
    @Override
    public Command moveToPositionCommand(Supplier<Climber.Constants.Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().pos).withName("climber.moveToPosition");
    }

    /**
     * Sets the goal and moves the climber motors to the supplied position.
     * 
     * @param goalPositionSupplier The supplier for the goal position, which must
     *                             return a double
     * @return Command that will move to a specified position
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return runOnce(() -> {
            Constants.climberMotorLeft.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
            Constants.climberMotorRight.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
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
     *         {@link Constants.Position#RESET_POSITION the reset position}
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
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
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
            Constants.climberMotorLeft.stopMotor();
            Constants.climberMotorRight.stopMotor();
        }).andThen(() -> {
            Constants.climberMotorLeft.setNeutralMode(NeutralModeValue.Coast);
            Constants.climberMotorRight.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            Constants.climberMotorLeft.setNeutralMode(NeutralModeValue.Brake);
            Constants.climberMotorRight.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("climber.coastMotors");
    }
}

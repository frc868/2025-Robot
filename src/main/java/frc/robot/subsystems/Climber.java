package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

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

    private TalonFXConfigurator climberConfiguratorL = Constants.climberMotorLeft.getConfigurator();
    private TalonFXConfigurator climberConfiguratorR = Constants.climberMotorRight.getConfigurator();
    private CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    private FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    private MotorOutputConfigs outputConfigsL = new MotorOutputConfigs();
    private MotorOutputConfigs outputConfigsR = new MotorOutputConfigs();
    private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private Slot0Configs controlConfigs = new Slot0Configs();

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(Constants.Position.RESET_POSITION.pos);

    public Climber() {
        limitConfigs.SupplyCurrentLimit = Constants.CURRENT_LIMIT; // Create current limits
        limitConfigs.SupplyCurrentLimitEnable = true;
        climberConfiguratorL.apply(limitConfigs); // Applies current limits
        climberConfiguratorR.apply(limitConfigs);

        feedbackConfigs.SensorToMechanismRatio = Constants.GEAR_RATIO;
        climberConfiguratorL.apply(feedbackConfigs);
        climberConfiguratorR.apply(feedbackConfigs);

        outputConfigsL.Inverted = Constants.MOTOR_L_INVERTED;
        outputConfigsR.Inverted = Constants.MOTOR_R_INVERTED;
        climberConfiguratorL.apply(outputConfigsL);
        climberConfiguratorR.apply(outputConfigsR);

        controlConfigs.kP = Constants.PID.kP;
        controlConfigs.kI = Constants.PID.kI;
        controlConfigs.kD = Constants.PID.kD;

        controlConfigs.kA = Constants.Feedforward.kA;
        controlConfigs.kS = Constants.Feedforward.kS;
        controlConfigs.kV = Constants.Feedforward.kV;

        motionMagicConfigs.MotionMagicAcceleration = Constants.Feedforward.MM_ACCEL;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Feedforward.MM_CRUISE;
        motionMagicConfigs.MotionMagicJerk = Constants.Feedforward.MM_JERK;

        climberConfiguratorL.apply(controlConfigs);
        climberConfiguratorR.apply(controlConfigs);

        climberConfiguratorL.apply(motionMagicConfigs);
        climberConfiguratorR.apply(motionMagicConfigs);

    }

    @Override
    public double getPosition() {
        return Constants.climberMotorLeft.getPosition(true).getValueAsDouble();
        // Returns the position of the left climber motor
    }

    @Override
    public void resetPosition() {
        // Resets the position of the climber mechanism to the stated position set in
        // constants
        Constants.climberMotorLeft.setPosition(Constants.Position.RESET_POSITION.pos);
        Constants.climberMotorRight.setPosition(Constants.Position.RESET_POSITION.pos);
    }

    @Override
    public void setVoltage(double voltage) {
        // Set the voltage of the climber motors to a value clamped between -12V and 12V
        Constants.climberMotorLeft.setVoltage(MathUtil.clamp(voltage, -12, 12));
        Constants.climberMotorRight.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return moveToArbitraryPositionCommand(() -> mmRequest.Position);
    }

    @Override
    public Command moveToPositionCommand(Supplier<Climber.Constants.Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().pos);
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return runOnce(() -> {
            Constants.climberMotorLeft.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
            Constants.climberMotorRight.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
        });
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> delta.get() + getPosition());
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return moveToArbitraryPositionCommand(() -> getPosition());
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition);
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}

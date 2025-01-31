package frc.robot.subsystems;

import java.util.function.Supplier;

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
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Constants.Intake_Constants.Position;

/** Intake subsystem which intakes algae from ground. */
public class Intake extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Intake.Constants.Intake_Constants.Position> {
    /** Constant values of intake subsystem. */
    public static final class Constants {
        public static final class CAN_IDs {
            public static final int ARM_MOTOR = 0; // TODO
            public static final int INTAKE_MOTOR = 0; // TODO
        }

        public static final class Arm_Constants {
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

            public static final double VEL_TOLERANCE = 0; //TODO find good value
            public static final double POS_TOLERANCE = .05;

            public static final double CURRENT_LIMIT = 0; // TODO

            public static final double ENCODER_CONVERSION_FACTOR = 0 * 2 * Math.PI; // TODO get conversion factor for encoder rotations -> radians (replace 0 with gear ratio)

            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO Clockwise_Positive or CounterClockwise_Positive
        }
        
        public static final class Intake_Constants {

            public static final double CURRENT_LIMIT = 0; // TODO

            public static final double ENCODER_CONVERSION_FACTOR = 0 * 2 * Math.PI; // TODO get conversion factor for encoder rotations -> radians (replace 0 with gear ratio)

            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO Clockwise_Positive or CounterClockwise_Positive

            public static final double VOLTAGE = 0; // TODO

            /** Positions that intake arm can be in. */
            public static enum Position {
                STOW(0),
                GROUND_INTAKE(0),
                MANIPULATOR_INTAKE(0);

                public final double position;

                private Position(final double position) {
                    this.position = position;
                }
            }
        }
    }

    private final TalonFX armMotor = new TalonFX(Constants.CAN_IDs.ARM_MOTOR);
    private final TalonFXConfigurator armConfigurator = armMotor.getConfigurator();
    private final TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    private final MotionMagicVoltage mmRequestArm = new MotionMagicVoltage(0);

    private final TalonFX intakeMotor = new TalonFX(Constants.CAN_IDs.INTAKE_MOTOR);
    private final TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    private final CurrentLimitsConfigs armCurrentLimitConfigs = new CurrentLimitsConfigs();
    private final CurrentLimitsConfigs intakeCurrentLimitConfigs = new CurrentLimitsConfigs();

    private final FeedbackConfigs armConfigFeedback = new FeedbackConfigs();
    private final FeedbackConfigs intakeConfigFeedback = new FeedbackConfigs();

    private final Slot0Configs armControlConfig = new Slot0Configs();

    private final MotionMagicConfigs mmArmConfig = new MotionMagicConfigs();

    private final MotorOutputConfigs armConfigOutput = new MotorOutputConfigs();
    private final MotorOutputConfigs intakeConfigOutput = new MotorOutputConfigs();

    public Intake() {
        armCurrentLimitConfigs.SupplyCurrentLimit = Constants.Arm_Constants.CURRENT_LIMIT;
        armCurrentLimitConfigs.SupplyCurrentLimitEnable = true;
        intakeCurrentLimitConfigs.SupplyCurrentLimit = Constants.Intake_Constants.CURRENT_LIMIT;
        intakeCurrentLimitConfigs.SupplyCurrentLimitEnable = true;
        armConfigs.CurrentLimits = armCurrentLimitConfigs;
        intakeConfigs.CurrentLimits = intakeCurrentLimitConfigs;

        armConfigOutput.Inverted = Constants.Arm_Constants.MOTOR_DIRECTION;
        intakeConfigOutput.Inverted = Constants.Intake_Constants.MOTOR_DIRECTION;
        armConfigs.MotorOutput = armConfigOutput;
        intakeConfigs.MotorOutput = intakeConfigOutput;

        armConfigFeedback.RotorToSensorRatio = Constants.Arm_Constants.ENCODER_CONVERSION_FACTOR;
        intakeConfigFeedback.RotorToSensorRatio = Constants.Intake_Constants.ENCODER_CONVERSION_FACTOR;
        armConfigs.Feedback = armConfigFeedback;
        intakeConfigs.Feedback = intakeConfigFeedback;

        armControlConfig.kP = Constants.Arm_Constants.PID.kP;
        armControlConfig.kI = Constants.Arm_Constants.PID.kI;
        armControlConfig.kD = Constants.Arm_Constants.PID.kD;
        armControlConfig.kS = Constants.Arm_Constants.FeedForward.kS;
        armControlConfig.kG = Constants.Arm_Constants.FeedForward.kG;
        armControlConfig.kV = Constants.Arm_Constants.FeedForward.kV;
        armControlConfig.kA = Constants.Arm_Constants.FeedForward.kA;
        armConfigs.Slot0 = armControlConfig;

        mmArmConfig.MotionMagicAcceleration = Constants.Arm_Constants.FeedForward.MM_ACCEL;
        mmArmConfig.MotionMagicCruiseVelocity = Constants.Arm_Constants.FeedForward.MM_CRUISE;
        mmArmConfig.MotionMagicJerk = Constants.Arm_Constants.FeedForward.MM_JERK;
        armConfigs.MotionMagic = mmArmConfig;

        armConfigurator.apply(armConfigs);
        intakeConfigurator.apply(intakeConfigs);
    }

    @Override
    public double getPosition() {
        return armMotor.getPosition(true).getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        armMotor.setPosition(Constants.Intake_Constants.Position.STOW.position);
    }

    @Override
    public void setVoltage(double voltage) {
        armMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setVoltageBar(double voltage) {
        intakeMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            armMotor.setControl(mmRequestArm.withPosition(mmRequestArm.Position));
        })
        .withName("intake.moveToCurrentGoalCommand"); 
    }

    public final boolean atGoal() {
        return Math.abs(mmRequestArm.Position - getPosition()) < Constants.Arm_Constants.POS_TOLERANCE && Math.abs(armMotor.getVelocity().getValueAsDouble()) < Constants.Arm_Constants.VEL_TOLERANCE;
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().position)
        .withName("intake.moveToPositionCommand");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
            runOnce(() -> {
                armMotor.setControl(mmRequestArm.withPosition(goalPositionSupplier.get()));
            }),
            moveToCurrentGoalCommand().until(this::atGoal)
        )
        .withName("intake.moveToArbitraryPositionCommand");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> mmRequestArm.Position + delta.get())
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
            intakeMotor.stopMotor();
            armMotor.stopMotor();
        })
        .andThen(() -> {
            armMotor.setNeutralMode(NeutralModeValue.Coast);
            intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        })
        .finallyDo((s) -> {
            armMotor.setNeutralMode(NeutralModeValue.Brake);
            intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("intake.coastMotorsCommand");
    }

    @Override
    public Command runRollersCommand() {
        return runOnce(() -> setVoltageBar(Constants.Intake_Constants.VOLTAGE)) // TODO implement voltage
        .withName("intake.runRollersCommand"); 
    }

    @Override
    public Command reverseRollersCommand() {
        return runOnce(() -> setVoltageBar(-Constants.Intake_Constants.VOLTAGE)) // TODO implement voltage
        .withName("intake.reverseRollersCommand"); 
    }
}

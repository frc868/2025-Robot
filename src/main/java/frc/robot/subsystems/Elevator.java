package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;

/** Subsystem which lifts manipulator subsystem. */
public class Elevator extends SubsystemBase
        implements BaseLinearMechanism<Elevator.Constants.Position> {
    /** Constant values of elevator subsystem. */
    public static final class Constants {
        private static final class CANIDs {
            private static final int elevatorMotorL = 0; // TODO get actual can ids
            private static final int elevatorMotorR = 0; // TODO get actual can ids
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

        public static final double MAX_AMPS = 10;
        public static final double RESET_POS = 0; // TODO get real value
        public static final double ENCODER_CONVERSION_FACTOR = 1; // conversion factor for encoder rotations -> linear
                                                                  // distance

        /** Positions that elevator subsystem can be in. */
        public enum Position {
            GROUND(0.0),
            L1(0.0),
            L2(0.0),
            L3(0.0),
            L4(0.0),
            CORAL_INTAKE(0.0),
            PROCESSOR(0.0);

            public double value;

            Position(double value) {
                this.value = value;
            }
        }
    }

    // Make motor objects
    private TalonFX elevatorMotorL = new TalonFX(Constants.CANIDs.elevatorMotorL);
    private TalonFX elevatorMotorR = new TalonFX(Constants.CANIDs.elevatorMotorR);

    // Get the motor configurators
    private TalonFXConfigurator elevatorConfigL = elevatorMotorL.getConfigurator();
    private TalonFXConfigurator elevatorConfigR = elevatorMotorR.getConfigurator();

    // Make the configs that are applied to motors later
    private CurrentLimitsConfigs elevatorConfig_Current = new CurrentLimitsConfigs();
    private FeedbackConfigs elevatorConfig_Feedback = new FeedbackConfigs();
    private Slot0Configs controlConfig = new Slot0Configs();
    private MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    // Make the Motion Magic object that sets where the motors should go
    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // Make the voltage object
    private VoltageOut voltageRequest = new VoltageOut(0);

    // Constructor (initialization)
    public Elevator() {
        // Current limit config application
        elevatorConfig_Current.SupplyCurrentLimit = Constants.MAX_AMPS;
        elevatorConfig_Current.SupplyCurrentLimitEnable = true;
        // Convert motor rotations to a linear unit of measure for height
        elevatorConfig_Feedback.SensorToMechanismRatio = Constants.ENCODER_CONVERSION_FACTOR;
        // Apply the configs
        elevatorConfigL.apply(elevatorConfig_Current);
        elevatorConfigL.apply(elevatorConfig_Feedback);
        elevatorConfigR.apply(elevatorConfig_Current);
        elevatorConfigR.apply(elevatorConfig_Feedback);

        // PID config application
        controlConfig.kP = Constants.PID.kP;
        controlConfig.kI = Constants.PID.kI;
        controlConfig.kD = Constants.PID.kD;
        controlConfig.kS = Constants.Feedforward.kS;
        controlConfig.kV = Constants.Feedforward.kV;
        controlConfig.kA = Constants.Feedforward.kA;

        elevatorConfigL.apply(controlConfig);
        elevatorConfigR.apply(controlConfig);

        // Motion Magic config application
        mmConfig.MotionMagicAcceleration = Constants.Feedforward.MM_ACCEL;
        mmConfig.MotionMagicCruiseVelocity = Constants.Feedforward.MM_CRUISE;
        mmConfig.MotionMagicCruiseVelocity = Constants.Feedforward.MM_JERK;

        elevatorConfigL.apply(mmConfig);
        elevatorConfigR.apply(mmConfig);
    }

    /**
     * Finds the position of the {@link #elevatorMotorL motor}
     * 
     * @return The position of the elevator (in whatever unit is used for the
     *         {@link Constants#ENCODER_CONVERSION_FACTOR conversion factor})
     */
    @Override
    public double getPosition() {
        return elevatorMotorL.getPosition(true).getValueAsDouble();
    }

    /**
     * Resets the position of the motors to a specified {@link Constants#RESET_POS
     * position} (Should be a hard stop)
     */
    @Override
    public void resetPosition() {
        elevatorMotorL.setPosition(Constants.RESET_POS);
        elevatorMotorR.setPosition(Constants.RESET_POS);
    }

    /**
     * Sets the voltage of the motors directly, {@link MathUtil#clamp clamped} to
     * safe values
     */
    @Override
    public void setVoltage(double voltage) {
        elevatorMotorL.setControl(voltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
        elevatorMotorR.setControl(voltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
    }

    /**
     * Moves the motors to the currently set goal
     * 
     * @return Command to move the motors to the already set goal
     */
    @Override
    public Command moveToCurrentGoalCommand() {
        return moveToArbitraryPositionCommand(() -> mmRequest.Position);
    }

    /**
     * Moves the motors to one of the main {@link Constants.Position positions}
     * 
     * @return Command that will move the motors to the setpoint enum
     */
    @Override
    public Command moveToPositionCommand(Supplier<Elevator.Constants.Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().value);
    }

    /**
     * Moves the motors to any position supplied
     * 
     * @return Command that will move the motors to the position supplied
     */
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return runOnce(() -> {
            elevatorMotorL.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
            elevatorMotorR.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
        });
    }

    /**
     * Moves the elevator's motors to a position relative to the current position.
     * 
     * @param delta A supplier of how much relative change is wanted, in meters.
     *              (Positive moves up)
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> mmRequest.Position + delta.get());
    }

    // Keep mechanism at current position
    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> {
            elevatorMotorL.setControl(mmRequest.withPosition(elevatorMotorL.getPosition(true).getValue()));
            elevatorMotorR.setControl(mmRequest.withPosition(elevatorMotorR.getPosition(true).getValue()));
        });
    }

    // Reset the position of the mechanism
    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition());
    }

    /**
     * Creates a command that will manually set the speed of the elevator's motors
     * 
     * @param speed Supplier for the speed (-1 to 1) to set the motor to
     * @return Command that sets the speed to the value returned by the supplier
     */
    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return run(() -> {
            setVoltage(speed.get() * 12.0);
        });
    }

    /**
     * Creates command to stop the elevator's motors and allow them to coast.
     * 
     * @return the command that allows motors to coast
     */
    @Override
    public Command coastMotorsCommand() {
        return new FunctionalCommand(
                () -> {
                    elevatorMotorL.setNeutralMode(NeutralModeValue.Coast);
                    elevatorMotorR.setNeutralMode(NeutralModeValue.Coast);
                },
                () -> {
                    elevatorMotorL.stopMotor();
                    elevatorMotorR.stopMotor();
                },
                (d) -> {
                    elevatorMotorL.setNeutralMode(NeutralModeValue.Brake);
                    elevatorMotorR.setNeutralMode(NeutralModeValue.Brake);
                },
                () -> false,
                this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

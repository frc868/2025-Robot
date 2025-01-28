package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem which lifts manipulator subsystem. */
public class Elevator extends SubsystemBase
        implements BaseLinearMechanism<Elevator.Constants.Position> {
    /** Constant values of elevator subsystem. */
    public static final class Constants {
        private static final class CANIDs {
            private static final int ELEVATOR_MOTOR_L = 0; // TODO get actual can ids
            private static final int ELEVATOR_MOTOR_R = 0; // TODO get actual can ids
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

        public static final InvertedValue L_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
                                                                                                // Clockwise_Positive or
                                                                                                // CounterClockwise_Positive
        public static final InvertedValue R_MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
                                                                                                // Clockwise_Positive or
                                                                                                // CounterClockwise_Positive
        public static final double MAX_AMPS = 10; // TODO find good # amps
        public static final double ENCODER_CONVERSION_FACTOR = 1; // TODO get conversion factor for encoder rotations ->
                                                                  // linear distance

        /** Positions that elevator subsystem can be in. */
        public static enum Position {
            RESET_POS(0.0), // TODO get actual position
            GROUND(0.0), // TODO get actual position
            L1(0.0), // TODO get actual position
            L2(0.0), // TODO get actual position
            L3(0.0), // TODO get actual position
            L4(0.0), // TODO get actual position
            CORAL_INTAKE(0.0), // TODO get actual position
            PROCESSOR(0.0); // TODO get actual position

            public final double value;

            Position(double value) {
                this.value = value;
            }
        }
    }

    // Make motor objects
    private final TalonFX elevatorMotorL = new TalonFX(Constants.CANIDs.ELEVATOR_MOTOR_L);
    private final TalonFX elevatorMotorR = new TalonFX(Constants.CANIDs.ELEVATOR_MOTOR_R);

    // Get the motor configurators
    private final TalonFXConfigurator elevatorConfigL = elevatorMotorL.getConfigurator();
    private final TalonFXConfigurator elevatorConfigR = elevatorMotorR.getConfigurator();

    // Make the configs that are applied to motors later
    private final CurrentLimitsConfigs elevatorConfigCurrent = new CurrentLimitsConfigs();
    private final FeedbackConfigs elevatorConfigFeedback = new FeedbackConfigs();
    private final Slot0Configs controlConfig = new Slot0Configs();
    private final MotionMagicConfigs mmConfig = new MotionMagicConfigs();
    private final MotorOutputConfigs elevatorConfigOutputL = new MotorOutputConfigs();
    private final MotorOutputConfigs elevatorConfigOutputR = new MotorOutputConfigs();

    // Make the Motion Magic object that sets where the motors should go
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // Make the voltage object
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Constructor (initialization)
    public Elevator() {
        // Current limit config application
        elevatorConfigCurrent.SupplyCurrentLimit = Constants.MAX_AMPS;
        elevatorConfigCurrent.SupplyCurrentLimitEnable = true;

        // Set inversions
        elevatorConfigOutputL.Inverted = Constants.L_MOTOR_DIRECTION;
        elevatorConfigOutputR.Inverted = Constants.R_MOTOR_DIRECTION;

        // Convert motor rotations to a linear unit of measure for height
        elevatorConfigFeedback.SensorToMechanismRatio = Constants.ENCODER_CONVERSION_FACTOR;

        // PID config application
        controlConfig.kP = Constants.PID.kP;
        controlConfig.kI = Constants.PID.kI;
        controlConfig.kD = Constants.PID.kD;
        controlConfig.kS = Constants.Feedforward.kS;
        controlConfig.kV = Constants.Feedforward.kV;
        controlConfig.kA = Constants.Feedforward.kA;

        // Motion Magic config application
        mmConfig.MotionMagicAcceleration = Constants.Feedforward.MM_ACCEL;
        mmConfig.MotionMagicCruiseVelocity = Constants.Feedforward.MM_CRUISE;
        mmConfig.MotionMagicJerk = Constants.Feedforward.MM_JERK;

        // Apply the configs
        elevatorConfigL.apply(elevatorConfigCurrent);
        elevatorConfigL.apply(elevatorConfigFeedback);
        elevatorConfigR.apply(elevatorConfigCurrent);
        elevatorConfigR.apply(elevatorConfigFeedback);
        elevatorConfigL.apply(elevatorConfigOutputL);
        elevatorConfigR.apply(elevatorConfigOutputR);
        elevatorConfigL.apply(controlConfig);
        elevatorConfigR.apply(controlConfig);
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
        elevatorMotorL.setPosition(Constants.Position.RESET_POS.value);
        elevatorMotorR.setPosition(Constants.Position.RESET_POS.value);
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
        return moveToArbitraryPositionCommand(() -> mmRequest.Position)
                .withName("elevator.moveToCurrentGoalCommand");
    }

    /**
     * Moves the motors to one of the main {@link Constants.Position positions}
     * 
     * @return Command that will move the motors to the setpoint enum
     */
    @Override
    public Command moveToPositionCommand(Supplier<Elevator.Constants.Position> goalPositionSupplier) {
        return moveToArbitraryPositionCommand(() -> goalPositionSupplier.get().value)
                .withName("elevator.moveToPositionCommand");
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
        })
                .withName("elevator.moveToArbitraryPositionCommand");
    }

    /**
     * Moves the elevator's motors to a position relative to the current position.
     * 
     * @param delta A supplier of how much relative change is wanted, in meters.
     *              (Positive moves up)
     */
    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> mmRequest.Position + delta.get())
                .withName("elevator.movePositionDeltaCommand");
    }

    /**
     * Creates a command that sets the elevator motor goal to the current position,
     * and moves to
     * that goal.
     * 
     * @return command to hold current position
     */
    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> {
            elevatorMotorL.setControl(mmRequest.withPosition(elevatorMotorL.getPosition(true).getValue()));
            elevatorMotorR.setControl(mmRequest.withPosition(elevatorMotorR.getPosition(true).getValue()));
        })
                .withName("elevator.holdCurrentPositionCommand");
    }

    /**
     * Creates an instantaneous command that resets the position of the elevator
     * mechanism.
     * 
     * @return command to reset position
     */
    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition())
                .withName("elevator.resetPositionCommand");

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
        })
                .withName("elevator.setOverridenSpeedCommand");

    }

    /**
     * Creates command to stop the elevator's motors and allow them to coast.
     * 
     * @return the command that allows motors to coast
     */
    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            elevatorMotorL.stopMotor();
            elevatorMotorR.stopMotor();
        }).andThen(() -> {
            elevatorMotorL.setNeutralMode(NeutralModeValue.Coast);
            elevatorMotorR.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            elevatorMotorL.setNeutralMode(NeutralModeValue.Brake);
            elevatorMotorR.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotors");
    }
}
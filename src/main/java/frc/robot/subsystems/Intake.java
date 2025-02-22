package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Constants.CAN;
import frc.robot.subsystems.Intake.Constants.Pivot;
import frc.robot.subsystems.Intake.Constants.Rollers;

/** Subsystem which intakes algae from ground. */
@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    /** Constant values of intake subsystem. */
    public static final class Constants {
        /** CAN information of intake motors. */
        public static final class CAN {
            /** CAN bus intake motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of intake motors. */
            public static final class IDs {
                /** CAN ID of intake pivot left motor. */
                private static final int PIVOT_LEFT = 16;
                /** CAN ID of intake pivot right motor. */
                // private static final int PIVOT_RIGHT = 17;
                /** CAN ID of intake rollers motor. */
                private static final int ROLLERS = 15;
            }
        }

        /** Constant values of intake pivot. */
        public static final class Pivot {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * pivot to be rotation away from zero point.
             */
            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
            public static final double GEAR_RATIO = 24.0 / 11.0;
            /** Ratio of motor rotations to intake pivot rotations. */
            public static final double SENSOR_TO_MECHANISM = GEAR_RATIO;
            /** Intake pivot motor current limit. */
            public static final double CURRENT_LIMIT = 40; // TODO
            public static final double VOLTAGE = 2;
        }

        /** Constant values of intake rollers. */
        public static final class Rollers {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * rollers to be rotation which intakes algae.
             */
            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TOD
            /** Intake rollers motor current limit. */
            public static final double CURRENT_LIMIT = 10; // TODO
            /** Voltage to run intake rollers motor at. */
            public static final double VOLTAGE = 6; // TODO
        }
    }

    /** Intake pivot left motor. */
    @Log
    private final TalonFX pivotMotor = new TalonFX(CAN.IDs.PIVOT_LEFT, CAN.BUS);
    /** Intake pivot motor configuration object. */
    private final TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
    /** Request object for setting motor current. */
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    /** Intake rollers motor. */
    @Log
    private final TalonFX rollersMotor = new TalonFX(CAN.IDs.ROLLERS, CAN.BUS);
    /** Intake rollers motor configuration object. */
    private final TalonFXConfiguration rollersMotorConfigs = new TalonFXConfiguration();
    /** Intake rollers motor current request object. */
    private final VoltageOut rollersVoltageRequest = new VoltageOut(0);

    /** Initialize intake pivot and rollers motor configurations. */
    public Intake() {
        pivotMotorConfigs.MotorOutput.Inverted = Pivot.MOTOR_DIRECTION;

        // pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = Pivot.CURRENT_LIMIT;

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);

        rollersMotorConfigs.MotorOutput.Inverted = Rollers.MOTOR_DIRECTION;

        // rollersMotorConfigs.CurrentLimits.SupplyCurrentLimit = Rollers.CURRENT_LIMIT;

        rollersMotor.getConfigurator().apply(rollersMotorConfigs);
    }

    /**
     * Sets the current of the intake's pivot
     * 
     * @param voltage The current to set the intake to
     */
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotVoltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)).withEnableFOC(true));
    }

    public Command extendPivotCommand() {
        return startEnd(() -> setPivotVoltage(Pivot.VOLTAGE), () -> setPivotVoltage(0));
    }

    public Command retractPivotCommand() {
        return startEnd(() -> setPivotVoltage(-Pivot.VOLTAGE), () -> setPivotVoltage(0));
    }

    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setPivotVoltage(speed.get() * 12), () -> setPivotVoltage(0))
                .withName("intake.setOverridenSpeedCommand");
    }

    @Override
    public Command runRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(-Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.reverseRollers");
    }

    public Command intakeGroundAlgaeCommand() {
        return extendPivotCommand().withTimeout(0.5).andThen(runRollersCommand()).withTimeout(0.5);
    }
}
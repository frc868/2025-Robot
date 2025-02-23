package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Manipulator.Constants.CAN;
import frc.robot.subsystems.Manipulator.Constants.Voltages;

import static frc.robot.subsystems.Manipulator.Constants.*;

/** Subsystem which intakes, holds, and scores scoring elements. */
@LoggedObject
public class Manipulator extends SubsystemBase implements BaseIntake {
    /** Constant values of manipulator subsystem. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * manipulator to be rotation which intakes a scoring element.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
        /** Manipulator motor current limit. */
        public static final double CURRENT_LIMIT = 100;
        public static final double CURRENT_DETECTION_THRESHOLD = 20; // TODO

        /** CAN information of manipulator motor. */
        public static final class CAN {
            /** CAN bus manipulator motor is on. */
            public static final String BUS = "rio";
            /** CAN ID of manipulator motor. */
            public static final int ID = 14;
        }

        /**
         * Voltages to set the manipulator motor to for various actions, in amps.
         */
        public enum Voltages {
            /** Voltage to run manipulator motor at to intake a scoring element. */
            INTAKE(9),
            /** Current to run manipulator motor at to score a scoring element. */
            SCORE(-4),
            /** Voltage to run manipulator motor at to hold a scoring element. */
            HOLD(4);

            /** Current to run manipulator motor at, in amps. */
            public final double voltage;

            private Voltages(final double voltage) {
                this.voltage = voltage;
            }
        };
    }

    /** Manipulator motor. */
    // @Log
    private final TalonFX motor = new TalonFX(CAN.ID, CAN.BUS);
    /** Manipulator motor configuration object. */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    /**
     * Manipulator motor torque current request object, using field oriented
     * control.
     */
    private final VoltageOut voltageRequest = new VoltageOut(0);
    @Log
    private final StatusSignal<Current> currentSignal = motor.getTorqueCurrent();
    /** Debouncer for filtering out current spike outliers. */
    private final Debouncer filter = new Debouncer(0.25);
    @Log
    private boolean temp;

    /** Initialize manipulator motor configurations. */
    public Manipulator() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        // motorConfigs.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;

        motor.getConfigurator().apply(motorConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);

        // SignalManager.register(currentSignal);

        setDefaultCommand(intakeScoringElementCommand());
    }

    /**
     * Checks the values of the motor current and compares them to a threshold to
     * identify if there is a game piece in the intake
     * 
     * @return Whether or not there is a game piece in the intake
     */
    @Log
    public boolean hasScoringElement() {
        return filter.calculate(motor.getTorqueCurrent().getValueAsDouble() > CURRENT_DETECTION_THRESHOLD);
    }

    @Override
    public Command runRollersCommand() {
        return runEnd(() -> motor.setControl(voltageRequest.withOutput(Voltages.INTAKE.voltage).withEnableFOC(true)),
                () -> motor.setControl(voltageRequest.withOutput(0))).withName("manipulator.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return runEnd(() -> motor.setControl(voltageRequest.withOutput(Voltages.SCORE.voltage).withEnableFOC(true)),
                () -> motor.setControl(voltageRequest.withOutput(0))).withName("manipulator.reverseRollers");
    }

    /**
     * Creates a command to hold a scoring element in the manipulator by applying a
     * voltage.
     * 
     * @return the command
     */
    public Command holdRollersCommand() {
        return run(() -> {
            System.out.println("Holding Rollers with Voltage: " + Voltages.HOLD.voltage);
            voltageRequest.withOutput(Voltages.HOLD.voltage).withEnableFOC(true);
        }).withName("manipulator.holdRollers");
    }

    /**
     * Creates a command to intake a scoring element and then hold it.
     * 
     * @return the command
     */
    public Command intakeScoringElementCommand() {
        return runRollersCommand().until(this::hasScoringElement).andThen(holdRollersCommand())
                .withName("manipulator.intakeScoringElement");
    }
}
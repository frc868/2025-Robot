package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Manipulator.Constants.*;

/** Subsystem which intakes and scores scoring elements. */
public class Manipulator extends SubsystemBase implements BaseIntake {
    /** Constant values of manipulator subsystem. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * manipulator pivot to be rotation away from zero point.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive;
        /** Manipulator motor current limit. */
        public static final double CURRENT_LIMIT = 0;
        /**
         * Current threshold which manipulator motor exceeds when intaking a scoring
         * element.
         */
        public static final double SCORING_ELEMENT_CURRENT_DETECTION_THRESHOLD = 0;

        /** CAN information of manipulator motor. */
        public static final class CAN {
            /** CAN bus manipulator motor is on. */
            public static final String BUS = "rio";
            /** CAN ID of manipulator motor. */
            public static final int ID = 14;
        }

        /**
         * Voltages to set the manipulator motor to, each accomplishing a different task
         */
        public enum Currents {
            /** The voltage that will hold a game piece in place inside the intake */
            HOLD(0.0),
            /** The voltage that will be used to intake a game piece */
            INTAKE(80),
            /** The voltage that will be used to push out a game piece */
            OUTTAKE(-20);

            /** This is used to access the values each name corresponds to, in volts */
            public final double current;

            private Currents(final double current) {
                this.current = current;
            }
        };
    }

    /** Manipulator motor. */
    private final TalonFX motor = new TalonFX(CAN.ID, CAN.BUS);
    /** Manipulator motor configuration object. */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    /** Stator current status signal of manipulator motor. */
    private final StatusSignal<Current> motorTorqueCurrent = motor.getTorqueCurrent();
    /** Manipulator motor voltage request object. */
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

    /** Debouncer for filtering out current spike outliers. */
    private final Debouncer currentSpikeDebouncer = new Debouncer(0.2);

    /** Initialize manipulator motor configurations. */
    public Manipulator() {
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 100;
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motor.getConfigurator().apply(motorConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the manipulator motor's voltage to the input provided
     * 
     * @param voltage The voltage to set the motor to (clamped to be between -12 and
     *                12)
     * @return Command that will set the voltage to the input provided once
     */
    public Command setVoltageCommand(double voltage) {
        return runEnd(() -> {
            motor.setControl(torqueCurrentRequest.withOutput(MathUtil.clamp(voltage, -12, 12)));
        }, () -> motor.setControl(torqueCurrentRequest.withOutput(0))).withName("manipulator.setVoltage");
    }

    /**
     * Creates command that will run the manipulator rollers forwards
     * 
     * @return A command that will run the rollers at
     *         {@link Constants#INTAKE_VOLTAGE a specified voltage}
     */
    @Override
    public Command runRollersCommand() {
        return runEnd(() -> motor.setControl(torqueCurrentRequest.withOutput(Currents.INTAKE.current)),
                () -> motor.setControl(torqueCurrentRequest.withOutput(0))).withName("manipulator.runRollers");
    }

    /**
     * Creates a command that will run the manipulator rollers backwards
     * 
     * @return A command that will run the rollers backwards at
     *         {@link Constants#INTAKE_VOLTAGE a specified voltage}
     */
    @Override
    public Command reverseRollersCommand() {
        return runEnd(() -> motor.setControl(torqueCurrentRequest.withOutput(Currents.INTAKE.current)),
                () -> motor.setControl(torqueCurrentRequest.withOutput(0))).withName("manipulator.reverseRollers");
    }

    /**
     * Creates a command that will stop the manipulator rollers
     * 
     * @return Command that will zero the voltage of the manipulator motor
     */
    public Command stopRollersCommand() {
        return setVoltageCommand(0).withName("manipulator.stopRollers");
    }

    /**
     * Sets the manipulator motor {@link Constants#HOLD_VOLTAGE voltage} to keep
     * game pieces in place
     * 
     * @return Command that keeps the intaked game pieces in place
     */
    public Command holdRollersCommand() {
        return run(() -> torqueCurrentRequest.withOutput(0)).withName("manipulator.holdRollers");
    }

    /**
     * Command that will run the rollers until a game piece is detected.
     * This piece will be detected by checking for current spikes in the motor
     * caused by increased load
     * 
     * @return Command that will {@link #runRollersCommand() intake} until the robot
     *         {@link #hasGamePiece() has a game piece}
     */
    public Command intakeGamePieceCommand() {
        return (runRollersCommand().repeatedly().until(this::hasGamePiece).andThen(holdRollersCommand()))
                .withName("manipulator.intakeGamePiece");
    }

    /**
     * Checks the values of the motor current and compares them to a threshold to
     * identify if there is a game piece in the intake
     * 
     * @return Whether or not there is a game piece in the intake
     */
    public boolean hasGamePiece() {
        motorTorqueCurrent.refresh();
        // If motor current > threshold for some specific time, return true
        return currentSpikeDebouncer.calculate(
                motorTorqueCurrent.getValueAsDouble() > Constants.SCORING_ELEMENT_CURRENT_DETECTION_THRESHOLD);
    }

}
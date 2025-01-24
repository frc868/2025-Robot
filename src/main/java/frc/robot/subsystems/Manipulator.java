package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem which intakes and scores scoring elements. */
public class Manipulator extends SubsystemBase implements BaseIntake {
    // create motor object
    private TalonFX manipulatorMotor;
    // create motor configuration object & each specific config
    private TalonFXConfigurator manipulatorConfigurator;
    private CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    private MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    // used to make sure that checking for game pieces is consistent and unaffected
    // by spikes
    private Debouncer intakeDebouncer = new Debouncer(0.2);
    // the object used to actually check the manipulator's current (to see if we
    // have game pieces)
    private StatusSignal<Current> manipulatorCurrent;

    // Constant values of manipulator subsystem.
    public static final class Constants {
        // CAN IDs for the Manipulator Motor
        public static final class CANIDS {
            public static final int MANIPULATOR_MOTOR_CANID = 0;
        }

        // Maximum current limit for the manipulator motor
        public static final double CURRENT_LIMIT = 0;
        // If the motor exerts force against a coral, it will require more current.
        // This value is the threshold that allows us to know if the motor has a piece
        public static final double TORQUE_CURRENT_DETECTION_THRESHOLD = 0;
        // Gear ratio in case we use pid/feedforward on this
        public static final double GEAR_RATIO = 0;
        // whether or not the motor must be inverted, depending on its placement
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
        // voltage to run the intake and outtake at
        public static final double INTAKE_VOLTAGE = 0;
        // voltage that will keep the game piece in place
        public static final double HOLD_VOLTAGE = 0;
    }

    public Manipulator() {
        // assign the manipulator motor to the specified CAN ID
        manipulatorMotor = new TalonFX(Constants.CANIDS.MANIPULATOR_MOTOR_CANID);

        // create current limits
        limitConfigs.SupplyCurrentLimit = Constants.CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLimitEnable = true;
        // and then apply the current limits
        manipulatorConfigurator.apply(limitConfigs);

        // create motor inversions
        outputConfigs.Inverted = Constants.INVERTED;
        // and then apply them
        manipulatorConfigurator.apply(outputConfigs);

        // use this to get the torque current in order to see if we have a game piece
        manipulatorCurrent = manipulatorMotor.getTorqueCurrent();

    }

    /**
     * Sets the manipulator motor's voltage to the input provided
     * 
     * @param voltage The voltage to set the motor to (clamped to be between -12 and
     *                12)
     * @return Command that will set the voltage to the input provided once
     */
    public Command setVoltageCommand(double voltage) {
        return this.runOnce(() -> {
            manipulatorMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
        });
    }

    /**
     * Creates command that will run the manipulator rollers forwards
     * 
     * @return A command that will run the rollers at
     *         {@link Constants#INTAKE_VOLTAGE a specified voltage}
     */
    @Override
    public Command runRollersCommand() {
        return setVoltageCommand(Constants.INTAKE_VOLTAGE);
    }

    /**
     * Creates a command that will run the manipulator rollers backwards
     * 
     * @return A command that will run the rollers backwards at
     *         {@link Constants#INTAKE_VOLTAGE a specified voltage}
     */
    @Override
    public Command reverseRollersCommand() {
        return setVoltageCommand(-Constants.INTAKE_VOLTAGE);
    }

    /**
     * Creates a command that will stop the manipulator rollers
     * 
     * @return Command that will zero the voltage of the manipulator motor
     */
    public Command stopRollersCommand() {
        return setVoltageCommand(0);
    }

    /**
     * Sets the manipulator motor {@link Constants#HOLD_VOLTAGE voltage} to keep
     * game pieces in place
     * 
     * @return Command that keeps the intaked game pieces in place
     */
    public Command holdRollersCommand() {
        return setVoltageCommand(Constants.HOLD_VOLTAGE);
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
        return runRollersCommand().repeatedly().until(this::hasGamePiece).andThen(stopRollersCommand());
    }

    /**
     * Checks the values of the motor current and compares them to a threshold to
     * identify if there is a game piece in the intake
     * 
     * @return Whether or not there is a game piece in the intake
     */
    public boolean hasGamePiece() {
        manipulatorCurrent.refresh();
        return intakeDebouncer.calculate(
                manipulatorCurrent.getValueAsDouble() > Constants.TORQUE_CURRENT_DETECTION_THRESHOLD);
    }

}
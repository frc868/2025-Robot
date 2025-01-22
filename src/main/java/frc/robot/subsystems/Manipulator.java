package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;

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

    @Override
    public Command runRollersCommand() {
        // TODO Auto-generated method stubs
        throw new UnsupportedOperationException("Unimplemented method 'runRollersCommand'");
    }

    @Override
    public Command reverseRollersCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reverseRollersCommand'");
    }
}

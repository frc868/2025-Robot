package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        /** CAN information of manipulator motor. */
        public static final class CAN {
            /** CAN bus manipulator motor is on. */
            public static final String BUS = "rio";
            /** CAN ID of manipulator motor. */
            public static final int ID = 14;
        }

        /**
         * Currents to set the manipulator motor to for various actions, in amps.
         */
        public enum Currents {
            /** Current to run manipulator motor at to intake a scoring element. */
            INTAKE(80),
            /** Current to run manipulator motor at to score a scoring element. */
            SCORE(-40),
            /** Current to run manipulator motor at to hold a scoring element. */
            HOLD(0.0); // TODO

            /** Current to run manipulator motor at, in amps. */
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
    /**
     * Manipulator motor torque current request object, using field oriented
     * control.
     */
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

    /** Debouncer for filtering out current spike outliers. */
    private final LinearFilter filter = LinearFilter.backwardFiniteDifference(1, 2, 0.25);

    /** Initialize manipulator motor configurations. */
    public Manipulator() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;

        motor.getConfigurator().apply(motorConfigs);

        motor.setNeutralMode(NeutralModeValue.Brake);

        // setDefaultCommand(intakeScoringElementCommand());
    }

    /**
     * Checks the values of the motor current and compares them to a threshold to
     * identify if there is a game piece in the intake
     * 
     * @return Whether or not there is a game piece in the intake
     */
    public boolean hasScoringElement() { // TODO
        motor.getVelocity().refresh();

        double temp = filter.calculate(motor.getVelocity().getValueAsDouble());
        System.out.println("Temp: " + temp);
        DriverStation.reportWarning("Temps: " + temp, false);
        return false;
    }

    @Override
    public Command runRollersCommand() {
        return runEnd(() -> motor.setControl(torqueCurrentRequest.withOutput(Currents.INTAKE.current)),
                () -> motor.setControl(torqueCurrentRequest.withOutput(0))).withName("manipulator.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return runEnd(() -> motor.setControl(torqueCurrentRequest.withOutput(Currents.SCORE.current)),
                () -> motor.setControl(torqueCurrentRequest.withOutput(0))).withName("manipulator.reverseRollers");
    }

    /**
     * Creates a command to hold a scoring element in the manipulator by applying a
     * current.
     * 
     * @return the command
     */
    public Command holdRollersCommand() {
        return run(() -> torqueCurrentRequest.withOutput(Currents.HOLD.current)).withName("manipulator.holdRollers");
    }

    /**
     * Creates a command to intake a scoring element and then hold it.
     * 
     * @return the command
     */
    public Command intakeScoringElementCommand() {
        return runRollersCommand().until(this::hasScoringElement).andThen(holdRollersCommand())
                .withName("manipulator.intakeGamePiece");
    }
}
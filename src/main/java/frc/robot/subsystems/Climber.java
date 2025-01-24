package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** Climber subsystem which hangs robot from deep cage. */
public class Climber extends SubsystemBase implements BaseSingleJointedArm<Climber.Constants.Position> {
    /** Constant values of climber subsystem. */
    private TalonFX climberMotor;
    private TalonFXConfigurator climberConfigurator;
    private CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    private StatusSignal<Current> climberCurrent;

    public static final class Constants {
        // Declares motor CanIDs
        public static final class CANIDS {
            public static final int CLIMBER_MOTOR_LEFT_CANID = 0; // Have not asked what the CanID should be yet
            public static final int CLIMBER_MOTOR_RIGHT_CANID = 0; // Also have not asked what the CanID should be yet
        }

        public static final double MAX_CURRENT_LIMIT = 0; // Max current limit for climber

        // "Torque - we are not sure if we need it yet" - Sage Ryker and Noor(but
        // written by Sage Ryker)

        public static final double GEAR_RATIO = 0;

        public static final double VOLTAGE = 0;

        /** Positions that climber subsystem can be in. */
        public enum Position {
        }
    }

    public Climber() {
        // Assign both climber motors to their specified CANID
        private TalonFX climberMotorLeft = new TalonFX(Constants.CLIMBER_MOTOR_LEFT_CANID);
        private TalonFX climberMotorRight = new TalonFX(Constants.CLIMBER_MOTOR_RIGHT_CANID);

        limit.SupplyCurrentLimit = Constants.CURRENT_LIMIT; // Create current limits
        limit.SupplyCurrentLimitEnable = true;
        climberConfigurator.apply(limit); // Applies current limits
    }

    @Override
    public double getPosition(double position) {
        return climberMotorLeft.getPosition(true).getValueAsDouble(); // Returns the position of the left climber motor
    }

    @Override
    public void resetPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Climber.Constants.Position> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
                "Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
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

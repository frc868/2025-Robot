package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;

/** Subsystem which lifts manipulator subsystem. */
public class Elevator extends SubsystemBase
        implements BaseLinearMechanism<Elevator.Constants.Position> {
    /** Constant values of elevator subsystem. */
    public static final class Constants {
        private static final class CANIDs {
            private static final int elevatorMotorL = 0; //TODO get actual can ids 
            private static final int elevatorMotorR = 0; //TODO get actual can ids 
        }

        public static class PID {
            public static final double kP = 0; //TODO find good value
            public static final double kI = 0; //TODO find good value
            public static final double kD = 0; //TODO find good value
        }

        public static class Feedforward {
            public static final double kS = 0; //TODO find good value
            public static final double kV = 0; //TODO find good value
            public static final double kA = 0; //TODO find good value
        }

        public static final double MAX_AMPS = 10;
        public static final double RESET_POS = 0; //TODO get real value

        /** Positions that elevator subsystem can be in. */
        public enum Position {
        }
    }

    //Make motors and config objects
    private TalonFX elevatorMotorL = new TalonFX(Constants.CANIDs.elevatorMotorL);
    private TalonFX elevatorMotorR = new TalonFX(Constants.CANIDs.elevatorMotorR);
    private TalonFXConfigurator elevatorConfigL = elevatorMotorL.getConfigurator();
    private TalonFXConfigurator elevatorConfigR = elevatorMotorR.getConfigurator();
    private CurrentLimitsConfigs elevatorConfig_Current = new CurrentLimitsConfigs();

    //Make controllers
    private Slot0Configs controlConfig = new Slot0Configs();

    // Constructor (initialization)
    public Elevator() {
        elevatorConfig_Current.SupplyCurrentLimit = Constants.MAX_AMPS;
        elevatorConfig_Current.SupplyCurrentLimitEnable = true;
        elevatorConfigL.apply(elevatorConfig_Current);
        elevatorConfigR.apply(elevatorConfig_Current);

        controlConfig.kP = Constants.PID.kP;
        controlConfig.kI = Constants.PID.kI;
        controlConfig.kD = Constants.PID.kD;
        controlConfig.kS = Constants.Feedforward.kS;
        controlConfig.kV = Constants.Feedforward.kV;
        controlConfig.kA = Constants.Feedforward.kA;
        elevatorConfigL.apply(controlConfig);
        elevatorConfigR.apply(controlConfig);

    }

    @Override
    public double getPosition() {
        return elevatorMotorL.getPosition(true).getValueAsDouble();
        //throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void resetPosition() {
        elevatorMotorL.setPosition(Constants.RESET_POS);
        elevatorMotorR.setPosition(Constants.RESET_POS);
        //throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMotorL.setVoltage(MathUtil.clamp(voltage, -12, 12));
        elevatorMotorR.setVoltage(MathUtil.clamp(voltage, -12, 12));
        //throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Elevator.Constants.Position> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
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
        return runOnce(() -> resetPosition());
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
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

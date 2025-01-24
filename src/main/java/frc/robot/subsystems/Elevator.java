package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
            public static final double MM_ACCEL = 0; //TODO find good value
            public static final double MM_CRUISE = 0; //TODO find good value
            public static final double MM_JERK = 0; //TODO find good value
        }

        public static final double MAX_AMPS = 10;
        public static final double RESET_POS = 0; //TODO get real value
        public static final double ENCODER_CONVERSION_FACTOR = 1; //conversion factor for encoder rotations -> linear distance

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
    private FeedbackConfigs elevatorConfig_Feedback = new FeedbackConfigs();
    private Slot0Configs controlConfig = new Slot0Configs();
    private MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    private MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // Constructor (initialization)
    public Elevator() {
        //Current limit config application
        elevatorConfig_Current.SupplyCurrentLimit = Constants.MAX_AMPS;
        elevatorConfig_Current.SupplyCurrentLimitEnable = true;
        elevatorConfig_Feedback.RotorToSensorRatio = Constants.ENCODER_CONVERSION_FACTOR;
        elevatorConfigL.apply(elevatorConfig_Current);
        elevatorConfigL.apply(elevatorConfig_Feedback);
        elevatorConfigR.apply(elevatorConfig_Current);
        elevatorConfigR.apply(elevatorConfig_Feedback);

        //PID config application
        controlConfig.kP = Constants.PID.kP;
        controlConfig.kI = Constants.PID.kI;
        controlConfig.kD = Constants.PID.kD;
        controlConfig.kS = Constants.Feedforward.kS;
        controlConfig.kV = Constants.Feedforward.kV;
        controlConfig.kA = Constants.Feedforward.kA;
        elevatorConfigL.apply(controlConfig);
        elevatorConfigR.apply(controlConfig);

        //Motion Magic config application
        mmConfig.MotionMagicAcceleration = Constants.Feedforward.MM_ACCEL;
        mmConfig.MotionMagicCruiseVelocity = Constants.Feedforward.MM_CRUISE;
        mmConfig.MotionMagicCruiseVelocity = Constants.Feedforward.MM_JERK;
        elevatorConfigL.apply(mmConfig);
        elevatorConfigR.apply(mmConfig);
    }

    // Return the current position in linear distance (if ENCODER_CONVERSION_FACTOR is properly set)
    @Override
    public double getPosition() {
        return elevatorMotorL.getPosition(true).getValueAsDouble();
        //throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    //Reset the encoder positions to a defined constant
    @Override
    public void resetPosition() {
        elevatorMotorL.setPosition(Constants.RESET_POS);
        elevatorMotorR.setPosition(Constants.RESET_POS);
        //throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    //Set the voltage (and by proxy speed) of the 2 motors
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

    //Move to any position
    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return runOnce(() -> {
            elevatorMotorL.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
            elevatorMotorR.setControl(mmRequest.withPosition(goalPositionSupplier.get()));
        });
        //throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    //Keep mechanism at current position
    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> {
            elevatorMotorL.setControl(mmRequest.withPosition(elevatorMotorL.getPosition(true).getValue()));
            elevatorMotorR.setControl(mmRequest.withPosition(elevatorMotorR.getPosition(true).getValue()));
        });
        //throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    //Reset the position of the mechanism
    @Override
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition());
        //throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    //Motors stop trying to brake until command ends, then they are set back to brake mode
    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            elevatorMotorL.stopMotor();
            elevatorMotorR.stopMotor();
            elevatorMotorL.setNeutralMode(NeutralModeValue.Coast);
            elevatorMotorR.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            elevatorMotorL.setNeutralMode(NeutralModeValue.Brake);
            elevatorMotorR.setNeutralMode(NeutralModeValue.Brake);
        });
        //throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}

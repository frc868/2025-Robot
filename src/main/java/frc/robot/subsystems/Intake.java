package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Constants.Position;

/** Intake subsystem which intakes algae from ground. */
public class Intake extends SubsystemBase implements BaseIntake, BaseSingleJointedArm<Intake.Constants.Position> {
    /** Constant values of intake subsystem. */
    public static final class Constants {
        public static final class CANIDS {
            public static final int INTAKE_MOTOR_LIFT_CANID = 0; // TO DO
            public static final int INTAKE_MOTOR_BAR_CANID = 0; // TO DO

        }

        public static final double CURRENT_LIMIT = 0; // TO DO

        public static final double VOLTAGE_BAR = 0; // TO DO

        public static final double VOLTAGE_LIFT = 0; // TO DO

        public static final double RESET_POSITION = 0; // Need a real value

        public enum Position {
        }
    }

    public Intake() {
        limitConfigs.SupplyCurrentLimit = Constants.CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLimitEnable = true;

        configLiftData.CurrentLimits = limitConfigs;
        configBarData.CurrentLimits = limitConfigs;

        configLift.apply(configLiftData);
        configBar.apply(configBarData);
    }

    private TalonFX intakeMotorLift = new TalonFX(Constants.CANIDS.INTAKE_MOTOR_LIFT_CANID);
    private TalonFX intakeMotorBar = new TalonFX(Constants.CANIDS.INTAKE_MOTOR_BAR_CANID);

    TalonFXConfigurator configLift = intakeMotorLift.getConfigurator();
    TalonFXConfiguration configLiftData = new TalonFXConfiguration();

    TalonFXConfigurator configBar = intakeMotorBar.getConfigurator();
    TalonFXConfiguration configBarData = new TalonFXConfiguration();

    private CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

    @Override
    public double getPosition() {
        return intakeMotorLift.getPosition(true).getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        // Resets the position of the climber mechanism to the stated position set in
        intakeMotorLift.setPosition(Constants.RESET_POSITION);
        intakeMotorBar.setPosition(Constants.RESET_POSITION);
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMotorLift.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setVoltageBar(double voltage) {
        intakeMotorBar.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Position> goalPositionSupplier) {
        throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        throw new UnsupportedOperationException("Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {

        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            intakeMotorBar.stopMotor();
            intakeMotorLift.stopMotor();
        })
                .andThen(() -> {
                    intakeMotorLift.setNeutralMode(NeutralModeValue.Coast);
                    intakeMotorBar.setNeutralMode(NeutralModeValue.Coast);
                })
                .finallyDo((s) -> {
                    intakeMotorLift.setNeutralMode(NeutralModeValue.Brake);
                    intakeMotorBar.setNeutralMode(NeutralModeValue.Brake);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("intake.coastMotors");
    }

    @Override
    public Command runRollersCommand() {
        return runOnce(() -> setVoltageBar(Constants.VOLTAGE_BAR)); // TO DO implement voltage
    }

    @Override
    public Command reverseRollersCommand() {
        return runOnce(() -> setVoltageBar(-Constants.VOLTAGE_BAR)); // TO DO implement voltage

    }
}

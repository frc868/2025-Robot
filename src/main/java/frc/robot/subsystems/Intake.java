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
        public static final class CAN_IDs {
            public static final int ARM_MOTOR = 0; // TO DO
            public static final int INTAKE_MOTOR = 0; // TO DO
        }

        public static final double CURRENT_LIMIT = 0; // TO DO

        public static final double INTAKE_VOLTAGE = 0; // TO DO

        /** Positions that intake arm can be in. */
        public static enum Position {
            STOW(0),
            GROUND_INTAKE(0),
            MANIPULATOR_INTAKE(0);

            public final double position;

            private Position(final double position) {
                this.position = position;
            }
        }
    }

    private final TalonFX armMotor = new TalonFX(Constants.CAN_IDs.ARM_MOTOR);
    private final TalonFXConfigurator armConfigurator = armMotor.getConfigurator();
    private final TalonFXConfiguration armConfigs = new TalonFXConfiguration();

    private final TalonFX intakeMotor = new TalonFX(Constants.CAN_IDs.INTAKE_MOTOR);
    private final TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();
    private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    private final CurrentLimitsConfigs armCurrentLimitConfigs = new CurrentLimitsConfigs();

    public Intake() {
        armCurrentLimitConfigs.SupplyCurrentLimit = Constants.CURRENT_LIMIT;
        armCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

        armConfigs.CurrentLimits = armCurrentLimitConfigs;
        intakeConfigs.CurrentLimits = armCurrentLimitConfigs;

        armConfigurator.apply(armConfigs);
        intakeConfigurator.apply(intakeConfigs);
    }

    @Override
    public double getPosition() {
        return armMotor.getPosition(true).getValueAsDouble();
    }

    @Override
    public void resetPosition() {
        armMotor.setPosition(Constants.RESET_POSITION);
    }

    @Override
    public void setVoltage(double voltage) {
        armMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    public void setVoltageBar(double voltage) {
        intakeMotor.setVoltage(MathUtil.clamp(voltage, -12, 12));
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
            intakeMotor.stopMotor();
            armMotor.stopMotor();
        })
                .andThen(() -> {
                    armMotor.setNeutralMode(NeutralModeValue.Coast);
                    intakeMotor.setNeutralMode(NeutralModeValue.Coast);
                })
                .finallyDo((s) -> {
                    armMotor.setNeutralMode(NeutralModeValue.Brake);
                    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
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

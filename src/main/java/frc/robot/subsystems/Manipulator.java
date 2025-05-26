package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.Arm.*;

@LoggedObject
public class Manipulator extends SubsystemBase implements BaseIntake {
    @Log
    private final TalonFX rollerMotor;

    private final VoltageOut rollerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Current> currentSignal;

    @Log
    public final Trigger hasGamePiece;
    @Log
    public final Trigger hasCoral;
    @Log
    public final Trigger hasAlgae;

    private boolean coralLatch = false;

    private final Debouncer filter = new Debouncer(0.3);

    private final NeutralOut stopRequest = new NeutralOut();

    private boolean manualCoralTrigger = false;

    @SendableLog
    private Command simulateCoralIntakeCommand = simulateCoralIntakeCommand();

    public Manipulator(Trigger isStowed) {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID, ROLLER_MOTOR_CAN_BUS);
        TalonFXConfigurator rollerMotorConfigurator = rollerMotor.getConfigurator();
        TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig.MotorOutput.Inverted = ROLLER_MOTOR_INVERSION;

        rollerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        if (RobotBase.isReal()) {
            rollerMotorConfig.CurrentLimits.StatorCurrentLimit = ROLLER_CURRENT_LIMIT;
            rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        rollerMotorConfigurator.apply(rollerMotorConfig);

        currentSignal = rollerMotor.getStatorCurrent();

        SignalManager.register(ROLLER_MOTOR_CAN_BUS, currentSignal);

        hasGamePiece = new Trigger(
                () -> filter.calculate((currentSignal.getValueAsDouble() > ROLLER_CURRENT_THRESHOLD)
                        && RobotBase.isReal()) || manualCoralTrigger);

        isStowed.and(hasGamePiece).onTrue(Commands.runOnce(() -> coralLatch = true));
        hasGamePiece.onFalse(Commands.runOnce(() -> coralLatch = false));

        hasCoral = new Trigger(() -> coralLatch);
        hasAlgae = isStowed.negate().and(hasGamePiece).and(hasCoral.negate());

        new Trigger(hasCoral).whileTrue(runRollersWithCoralCommand());

        setDefaultCommand(runRollersCommand());
    }

    @Override
    public Command runRollersCommand() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(10)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.runRollers");
    }

    public Command runRollersWithCoralCommand() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(1)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.runRollersWithCoral");
    }

    @Override
    public Command reverseRollersCommand() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(-7.5)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.reverseRollers");
    }

    public Command scoreNetCommand() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(-7.5)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.reverseRollers");
    }

    public Command scoreL1Command() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(-3)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.reverseRollers");
    }

    public Command scoreL23Command() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(-4)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.reverseRollers");
    }

    public Command groundIntakeAlgaeRollersCommand() {
        return startEnd(
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(8)),
                () -> rollerMotor.setControl(rollerVoltageRequest.withOutput(0)))
                .withName("manipulator.reverseRollers");
    }

    public Command stopRollersCommand() {
        return run(() -> {
            rollerMotor.setControl(stopRequest);
        }).withName("manipulator.stopRollersCommand");
    }

    public Command simulateCoralIntakeCommand() {
        return Commands.startEnd(() -> manualCoralTrigger = true, () -> manualCoralTrigger = false);
    }
}

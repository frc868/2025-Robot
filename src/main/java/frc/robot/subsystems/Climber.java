package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GlobalStates;

import static frc.robot.Constants.Climber.*;

import java.util.function.Supplier;

@LoggedObject
public class Climber extends SubsystemBase {
    @Log
    private final TalonFX leftMotor;
    @Log
    private final TalonFX rightMotor;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true).withUseTimesync(true);
    private final NeutralOut stopRequest = new NeutralOut().withUseTimesync(true);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;

    @Log
    private boolean initialized = RobotBase.isSimulation();

    @Log
    private double goalPosition;

    @SuppressWarnings("unused")
    private PositionTracker positionTracker;

    public Climber(PositionTracker positionTracker) {
        leftMotor = new TalonFX(LEFT_MOTOR_ID, MOTOR_CAN_BUS);
        TalonFXConfigurator leftMotorConfigurator = leftMotor.getConfigurator();
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = MOTOR_INVERSION;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            motorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        leftMotorConfigurator.apply(motorConfig);

        rightMotor = new TalonFX(RIGHT_MOTOR_ID, MOTOR_CAN_BUS);
        TalonFXConfigurator rightMotorConfigurator = rightMotor.getConfigurator();
        rightMotorConfigurator.apply(motorConfig);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));

        positionSignal = leftMotor.getPosition();
        velocitySignal = leftMotor.getVelocity();
        accelerationSignal = leftMotor.getAcceleration();
        voltageSignal = leftMotor.getMotorVoltage();

        SignalManager.register(
                MOTOR_CAN_BUS,
                positionSignal, velocitySignal, accelerationSignal, voltageSignal);

        this.positionTracker = positionTracker;

        positionTracker.addPositionSupplier("climber", this::getPosition);
    }

    @Log
    public Pose3d getComponentPose() {
        return new Pose3d(0, 0.32, 0.1105, new Rotation3d(0, 0, -Math.PI / 2.0));
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log
    public double getPosition() {
        return positionSignal.getValueAsDouble();
    }

    public boolean shouldEnforceSafeties(double intendedDirection) {
        if (Utils.applySoftStops(intendedDirection, getPosition(), MIN_POSITION_ROTATIONS,
                MAX_POSITION_ROTATIONS) == 0.0)
            return true;

        if (!GlobalStates.INITIALIZED.enabled()) {
            return true;
        }

        return false;
    }

    public void setVoltage(double voltage) {
        if (shouldEnforceSafeties(voltage)) {
            leftMotor.setControl(stopRequest);
        } else {
            leftMotor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    public Command climbCommand() {
        return Commands.startEnd(
                () -> setVoltage(-8),
                () -> setVoltage(0))
                .withName("climber.climb");
    }

    public Command declimbCommand() {
        return Commands.startEnd(
                () -> setVoltage(6),
                () -> setVoltage(0))
                .withName("climber.declimb");
    }

    public void resetPosition() {
        leftMotor.setPosition(0);
        initialized = true;
    }

    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("climber.resetPosition");
    }

    public Command manualInitializeCommand() {
        return Commands.runOnce(() -> initialized = true).ignoringDisable(true).withName("climber.setInitialized");
    }

    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("climber.setOverriddenSpeed");
    }
}

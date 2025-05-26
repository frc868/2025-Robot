package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.EqualsUtil;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlib.Utils;
import com.techhounds.houndutil.houndlib.subsystems.BaseLinearMechanism;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.GlobalStates;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.Elevator.*;

@LoggedObject
public class Elevator extends SubsystemBase implements BaseLinearMechanism<ElevatorPosition> {
    @Log
    private final TalonFX leftMotor;
    @Log
    private final TalonFX rightMotor;

    private ElevatorProfileParams profileParams = ElevatorProfileParams.MID;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true).withUseTimesync(true);
    private final DynamicMotionMagicVoltage positionRequest = new DynamicMotionMagicVoltage(
            0,
            profileParams.velocity / DRUM_CIRCUMFERENCE,
            profileParams.acceleration / DRUM_CIRCUMFERENCE,
            profileParams.jerk / DRUM_CIRCUMFERENCE)
            .withEnableFOC(true)
            .withUseTimesync(true);
    private final NeutralOut stopRequest = new NeutralOut().withUseTimesync(true);

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;

    @Log
    private boolean safetyTriggered = false;

    @Log
    private final ElevatorSim elevatorSim = new ElevatorSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MASS_KG,
            DRUM_RADIUS_METERS,
            MIN_HEIGHT_METERS,
            MAX_HEIGHT_METERS,
            true,
            ElevatorPosition.BOTTOM.value);

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidPositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    private final PositionTracker positionTracker;
    private final MechanismLigament2d ligament;

    @Log
    private boolean initialized = RobotBase.isSimulation();

    @Log
    private double goalPosition = 0;

    @Log
    private double simCurrent = 0;

    @Log
    public final Trigger isStowed;

    public Elevator(PositionTracker positionTracker, MechanismLigament2d ligament) {

        leftMotor = new TalonFX(LEFT_MOTOR_ID, MOTOR_CAN_BUS);
        TalonFXConfigurator leftMotorConfigurator = leftMotor.getConfigurator();
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = LEFT_MOTOR_INVERSION;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.SensorToMechanismRatio = GEARING;

        if (RobotBase.isReal()) {
            motorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        }

        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kG = kG;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.Slot1.kS = kS;
        motorConfig.Slot1.kG = kG_HIGH;
        motorConfig.Slot1.kV = kV;
        motorConfig.Slot1.kA = kA;
        motorConfig.Slot1.kP = kP;
        motorConfig.Slot1.kI = kI;
        motorConfig.Slot1.kD = kD;
        motorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = profileParams.velocity / DRUM_CIRCUMFERENCE;
        motorConfig.MotionMagic.MotionMagicAcceleration = profileParams.acceleration / DRUM_CIRCUMFERENCE;
        motorConfig.MotionMagic.MotionMagicJerk = profileParams.jerk / DRUM_CIRCUMFERENCE;

        leftMotorConfigurator.apply(motorConfig);

        rightMotor = new TalonFX(RIGHT_MOTOR_ID, MOTOR_CAN_BUS);
        TalonFXConfigurator rightMotorConfigurator = rightMotor.getConfigurator();
        motorConfig.MotorOutput.Inverted = RIGHT_MOTOR_INVERSION;
        rightMotorConfigurator.apply(motorConfig);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        positionSignal = leftMotor.getPosition();
        velocitySignal = leftMotor.getVelocity();
        accelerationSignal = leftMotor.getAcceleration();
        voltageSignal = leftMotor.getMotorVoltage();

        SignalManager.register(
                MOTOR_CAN_BUS,
                positionSignal, velocitySignal, accelerationSignal, voltageSignal);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds), Volts.of(5), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure
                                            .mut_replace(voltageSignal.getValueAsDouble(), Volts))
                                    .linearPosition(sysidPositionMeasure.mut_replace(getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), MetersPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;
        this.ligament = ligament;

        positionTracker.addPositionSupplier("elevator", this::getPosition);

        isStowed = new Trigger(() -> (positionTracker.getPosition("arm") > 2.5 && getPosition() < 0.1));
        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = leftMotor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // negative due to inversion state
        elevatorSim.setInputVoltage(-motorVoltage.in(Volts));
        elevatorSim.update(0.020);

        // set positions of the rotors by working back through units
        talonFXSim.setRawRotorPosition(-elevatorSim.getPositionMeters() / DRUM_CIRCUMFERENCE * GEARING);
        talonFXSim.setRotorVelocity(-elevatorSim.getVelocityMetersPerSecond() / DRUM_CIRCUMFERENCE * GEARING);

        ligament.setLength(getPosition());
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log(groups = "components")
    public Pose3d getStageComponentPose() {
        Transform3d transform = new Transform3d();
        if (getPosition() > STAGE_MOVEMENT_HEIGHT) {
            transform = new Transform3d(0, 0, getPosition() - STAGE_MOVEMENT_HEIGHT, new Rotation3d());
        }
        return new Pose3d(0.1747, 0, 0.0417, new Rotation3d()).plus(transform);
    }

    @Log(groups = "components")
    public Pose3d getCarriageComponentPose() {
        return new Pose3d(0.1747, 0, 0.0671 + getPosition(), new Rotation3d());
    }

    @Override
    @Log
    public double getPosition() {
        return positionSignal.getValueAsDouble() * DRUM_CIRCUMFERENCE;
    }

    @Log
    public double getVelocity() {
        return velocitySignal.getValueAsDouble() * DRUM_CIRCUMFERENCE;
    }

    @Override
    public void resetPosition() {
        leftMotor.setPosition(0);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        if (shouldEnforceSafeties(voltage)) {
            leftMotor.setControl(stopRequest);
        } else {
            leftMotor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    public boolean shouldEnforceSafeties(double intendedDirection) {
        if (Utils.applySoftStops(intendedDirection, getPosition(), MIN_HEIGHT_METERS,
                MAX_HEIGHT_METERS) == 0.0)
            return true;

        if (intendedDirection < 0
                && getPosition() < 0.70
                && getPosition() > 0.3
                && positionTracker.getPosition("arm") > 1.22) {
            return true;
        }
        if (intendedDirection > 0
                && getPosition() > 0.08
                && getPosition() < 0.50
                && positionTracker.getPosition("arm") > 1.5) {
            return true;
        }

        if (!GlobalStates.INITIALIZED.enabled()) {
            return true;
        }

        return false;
    }

    public void setElevatorProfileParams(ElevatorProfileParams params) {
        this.profileParams = params;
    }

    public Command setElevatorProfileParamsCommand(ElevatorProfileParams params) {
        return Commands.runOnce(() -> setElevatorProfileParams(params));
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            if (shouldEnforceSafeties(goalPosition - getPosition())) {
                leftMotor.setControl(stopRequest);
                safetyTriggered = true;
            } else {
                safetyTriggered = false;
                int slot = getPosition() > 0.795 ? 1 : 0;
                ElevatorProfileParams paramsToUse = profileParams;

                // override if the goal position is not L3
                if (goalPosition < 0.795 && goalPosition != 0.0) {
                    paramsToUse = ElevatorProfileParams.FAST;
                }
                leftMotor.setControl(positionRequest.withPosition(goalPosition / DRUM_CIRCUMFERENCE)
                        .withVelocity(paramsToUse.velocity / DRUM_CIRCUMFERENCE)
                        .withAcceleration(paramsToUse.acceleration / DRUM_CIRCUMFERENCE)
                        .withJerk(paramsToUse.jerk / DRUM_CIRCUMFERENCE)
                        .withSlot(slot));
            }
        }).withName("elevator.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get().value),
                moveToCurrentGoalCommand().until(this::atGoal))
                .withTimeout(3)
                .withName("elevator.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get()),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("elevator.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> goalPosition += delta.get())
                .withName("elevator.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> goalPosition = positionSignal.getValueAsDouble()).andThen(moveToCurrentGoalCommand())
                .withName("elevator.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("elevator.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("elevator.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(() -> setVoltage(0))
                .andThen(() -> {
                    leftMotor.setNeutralMode(NeutralModeValue.Coast);
                    rightMotor.setNeutralMode(NeutralModeValue.Coast);
                })
                .finallyDo((d) -> {
                    leftMotor.setNeutralMode(NeutralModeValue.Brake);
                    rightMotor.setNeutralMode(NeutralModeValue.Brake);
                    goalPosition = getPosition();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> goalPosition = getPosition()).withName("elevator.resetControllers");
    }

    @Log
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getPosition(), goalPosition, TOLERANCE);
    }

    public Command manualInitializeCommand() {
        return Commands.runOnce(() -> initialized = true).ignoringDisable(true).withName("elevator.setInitialized");
    }
}

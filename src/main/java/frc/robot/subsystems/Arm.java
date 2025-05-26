package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import com.techhounds.houndutil.houndlog.SignalManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.GlobalStates;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Arm.*;

@LoggedObject
public class Arm extends SubsystemBase implements BaseSingleJointedArm<ArmPosition> {
    @Log
    private final TalonFX motor;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut stopRequest = new NeutralOut();

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> voltageSignal;

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    @Log
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOI,
            COM_DISTANCE_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            ArmPosition.HORIZONTAL.value);

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidPositionMeasure = Rotations.mutable(0);
    private final MutAngularVelocity sysidVelocityMeasure = RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    private final PositionTracker positionTracker;
    private final MechanismLigament2d ligament;

    @Log
    private boolean initialized = RobotBase.isSimulation();

    @Log
    private double goalPosition;

    @Log
    private int gainSlot = 0;

    public Arm(PositionTracker positionTracker, MechanismLigament2d ligament) {

        motor = new TalonFX(MOTOR_ID, MOTOR_CAN_BUS);
        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = MOTOR_INVERSION;

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
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.Slot1.kS = kS;
        motorConfig.Slot1.kG = kG_CORAL;
        motorConfig.Slot1.kV = kV;
        motorConfig.Slot1.kA = kA;
        motorConfig.Slot1.kP = kP;
        motorConfig.Slot1.kI = kI;
        motorConfig.Slot1.kD = kD;
        motorConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.Slot2.kS = kS;
        motorConfig.Slot2.kG = kG_ALGAE;
        motorConfig.Slot2.kV = kV;
        motorConfig.Slot2.kA = kA;
        motorConfig.Slot2.kP = kP;
        motorConfig.Slot2.kI = kI;
        motorConfig.Slot2.kD = kD;
        motorConfig.Slot2.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = MAX_VELOCITY_ROTATIONS_PER_SECOND;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;

        motorConfigurator.apply(motorConfig);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();
        voltageSignal = motor.getMotorVoltage();

        motor.getClosedLoopReference().setUpdateFrequency(50);
        motor.getClosedLoopReferenceSlope().setUpdateFrequency(50);

        SignalManager.register(
                MOTOR_CAN_BUS,
                positionSignal, velocitySignal, accelerationSignal, voltageSignal);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            setVoltage(volts.in(Volts));
                        },
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(voltageSignal.getValueAsDouble(),
                                            Volts))
                                    .angularPosition(
                                            sysidPositionMeasure.mut_replace(positionSignal.getValueAsDouble(),
                                                    Rotations))
                                    .angularVelocity(sysidVelocityMeasure.mut_replace(velocitySignal.getValueAsDouble(),
                                            RotationsPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;
        this.ligament = ligament;

        positionTracker.addPositionSupplier("arm", this::getPosition);

        setDefaultCommand(moveToCurrentGoalCommand());
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = motor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(-motorVoltage.in(Volts));
        armSim.update(0.020);

        // negative because inversion state
        talonFXSim.setRawRotorPosition(-armSim.getAngleRads() / (2 * Math.PI) * GEARING);
        talonFXSim.setRotorVelocity(-armSim.getVelocityRadPerSec() / (2 * Math.PI) * GEARING);

        ligament.setAngle(Units.radiansToDegrees(getPosition()) + 270);
    }

    public boolean getInitialized() {
        return initialized;
    }

    @Log(groups = "components")
    public Pose3d getComponentPose() {
        // since 0 position needs to be horizontal
        return new Pose3d(0.275, 0, 0.435 + positionTracker.getPosition("elevator"), new Rotation3d(0, -0.45, 0))
                .plus(new Transform3d(new Translation3d(), new Rotation3d(0, -getPosition(), 0)));
    }

    public Transform3d getCoralTransform() {
        // since 0 position needs to be horizontal
        return new Transform3d(new Pose3d(),
                getComponentPose()
                        .plus(new Transform3d(0.3, 0, -0.1325, new Rotation3d(0, Units.degreesToRadians(-130), 0))));
    }

    public Transform3d getAlgaeTransform() {
        // since 0 position needs to be horizontal
        return new Transform3d(new Pose3d(),
                getComponentPose()
                        .plus(new Transform3d(0.42, 0, -0.3, new Rotation3d())));
    }

    @Log
    @Override
    public double getPosition() {
        return positionSignal.getValue().in(Radians);
    }

    @Log
    public double getVelocity() {
        return velocitySignal.getValue().in(RadiansPerSecond);
    }

    @Override
    public void resetPosition() {
        motor.setPosition(Units.radiansToRotations(ArmPosition.CORAL_INTAKE.value));
        initialized = true;
    }

    public boolean shouldEnforceSafeties(double intendedDirection) {
        if (Utils.applySoftStops(intendedDirection, getPosition(), MIN_ANGLE_RADIANS,
                MAX_ANGLE_RADIANS) == 0.0)
            return true;

        if (!GlobalStates.INITIALIZED.enabled()) {
            return true;
        }

        return false;
    }

    @Override
    public void setVoltage(double voltage) {
        if (shouldEnforceSafeties(voltage)) {
            motor.setControl(stopRequest);
        } else {
            motor.setControl(voltageRequest.withOutput(voltage));
        }
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            if (shouldEnforceSafeties(goalPosition - getPosition())) {
                motor.setControl(stopRequest);
            } else {
                motor.setControl(
                        positionRequest.withPosition(Radians.of(goalPosition).in(Rotations)).withSlot(gainSlot));
            }
        }).withName("arm.moveToCurrentGoal");
    }

    @Override
    public Command moveToPositionCommand(Supplier<ArmPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get().value),
                moveToCurrentGoalCommand().until(this::atGoal))
                .withTimeout(1)
                .withName("arm.moveToPosition");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> goalPosition = goalPositionSupplier.get()),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("arm.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> goalPosition + delta.get())
                .withName("arm.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> goalPosition = positionSignal.getValueAsDouble()).andThen(moveToCurrentGoalCommand())
                .withName("arm.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("arm.setOverriddenSpeed");
    }

    @Override
    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    motor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Coast;
                    motor.getConfigurator().apply(motorConfigs);
                })
                .finallyDo((d) -> {
                    MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
                    motor.getConfigurator().refresh(motorConfigs);
                    motorConfigs.NeutralMode = NeutralModeValue.Coast;
                    motor.getConfigurator().apply(motorConfigs);

                    goalPosition = getPosition();
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("arm.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("arm.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("arm.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> goalPosition = getPosition());
    }

    @Log
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(getPosition(), goalPosition, TOLERANCE);
    }

    public Command useCoralGainsCommand() {
        return Commands.startEnd(() -> gainSlot = 0, () -> gainSlot = 0).withName("arm.useCoralGains");
    }

    public Command useAlgaeGainsCommand() {
        return Commands.startEnd(() -> gainSlot = 0, () -> gainSlot = 0).withName("arm.useAlgaeGains");
    }

    public Command manualInitializeCommand() {
        return Commands.runOnce(() -> initialized = true).ignoringDisable(true).withName("arm.setInitialized");
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.Manipulator.Constants.CURRENT_DETECTION_THRESHOLD;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Constants.CAN;
import frc.robot.subsystems.Intake.Constants.Pivot;
import frc.robot.subsystems.Intake.Constants.Rollers;

/** Subsystem which intakes algae from ground. */
@LoggedObject
public class Intake extends SubsystemBase implements BaseIntake {
    /** Constant values of intake subsystem. */
    public static final class Constants {
        /** CAN information of intake motors. */
        public static final class CAN {
            /** CAN bus intake motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of intake motors. */
            public static final class IDs {
                /** CAN ID of intake pivot left motor. */
                private static final int PIVOT_LEFT = 16;
                /** CAN ID of intake pivot right motor. */
                // private static final int PIVOT_RIGHT = 17;
                /** CAN ID of intake rollers motor. */
                private static final int ROLLERS = 15;
            }
        }

        /** Constant values of intake pivot. */
        public static final class Pivot {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * pivot to be rotation away from zero point.
             */
            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;
            public static final double GEAR_RATIO = 24.0 / 11.0;
            /** Ratio of motor rotations to intake pivot rotations. */
            public static final double SENSOR_TO_MECHANISM = GEAR_RATIO;
            /** Intake pivot motor current limit. */
            public static final double CURRENT_LIMIT = 40; // TODO
            public static final double VOLTAGE = 2;
            public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getKrakenX60(2);
            public static final double MASS_KG = Units.lbsToKilograms(3);
            public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
            public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);

            public static final double MIN_ANGLE_RADIANS = Units.degreesToRadians(38);
            public static final double MAX_ANGLE_RADIANS = Units.degreesToRadians(143.81);

        }

        /** Constant values of intake rollers. */
        public static final class Rollers {
            /**
             * Direction of motor rotation defined as positive rotation. Defined for intake
             * rollers to be rotation which intakes algae.
             */
            public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TOD
            /** Intake rollers motor current limit. */
            public static final double CURRENT_LIMIT = 10; // TODO
            /** Voltage to run intake rollers motor at. */
            public static final double VOLTAGE = 6; // TODO
            public static final double CURRENT_DETECTION_THRESHOLD = 20;
        }
    }

    /** Intake pivot left motor. */
    @Log
    private final TalonFX pivotMotor = new TalonFX(CAN.IDs.PIVOT_LEFT, CAN.BUS);
    /** Intake pivot motor configuration object. */
    private final TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
    /** Request object for setting motor current. */
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);

    /** Intake rollers motor. */
    @Log
    private final TalonFX rollersMotor = new TalonFX(CAN.IDs.ROLLERS, CAN.BUS);
    /** Intake rollers motor configuration object. */
    private final TalonFXConfiguration rollersMotorConfigs = new TalonFXConfiguration();
    /** Intake rollers motor current request object. */
    private final VoltageOut rollersVoltageRequest = new VoltageOut(0);

    private final NeutralOut stopRequest = new NeutralOut();

    /** Debouncer for filtering out current spike outliers. */
    private final Debouncer filter = new Debouncer(0.25);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            Pivot.MOTOR_GEARBOX_REPR,
            Pivot.GEAR_RATIO,
            Pivot.MOI,
            Pivot.COM_DISTANCE_METERS,
            Pivot.MIN_ANGLE_RADIANS,
            Pivot.MAX_ANGLE_RADIANS,
            true,
            Units.degreesToRadians(135));

    /** Initialize intake pivot and rollers motor configurations. */
    public Intake() {
        pivotMotorConfigs.MotorOutput.Inverted = Pivot.MOTOR_DIRECTION;

        // pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = Pivot.CURRENT_LIMIT;

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);

        rollersMotorConfigs.MotorOutput.Inverted = Rollers.MOTOR_DIRECTION;

        // rollersMotorConfigs.CurrentLimits.SupplyCurrentLimit = Rollers.CURRENT_LIMIT;

        rollersMotor.getConfigurator().apply(rollersMotorConfigs);
    }

    @Override
    public void simulationPeriodic() {
        TalonFXSimState talonFXSim = pivotMotor.getSimState();
        Voltage motorVoltage = talonFXSim.getMotorVoltageMeasure();

        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(0.020);

        talonFXSim.setRawRotorPosition(armSim.getAngleRads() / (2 * Math.PI) * Pivot.GEAR_RATIO);
        talonFXSim.setRotorVelocity(armSim.getVelocityRadPerSec() / (2 * Math.PI) * Pivot.GEAR_RATIO);
    }

    @Log
    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    @Log
    public Pose3d getLeftFrontBarComponentPose() {
        return new Pose3d(0.343, 0.3645, 0.18, new Rotation3d(0, -getPosition(), 0));
    }

    @Log
    public Pose3d getRightFrontBarComponentPose() {
        return new Pose3d(0.343, -0.3645, 0.18, new Rotation3d(0, -getPosition(), 0));
    }

    @Log
    public Pose3d getTopBarComponentPose() {
        Pose3d backBarPose = new Pose3d(0.2157, 0, 0.18, new Rotation3d(0, -getPosition(), 0));
        Translation3d topBarTranslation = backBarPose.plus(new Transform3d(0.2573, 0, 0, new Rotation3d(0, 2.51, 0)))
                .getTranslation();
        return new Pose3d(topBarTranslation, new Rotation3d());
    }

    @Log
    public Pose3d getLeftBackBarComponentPose() {
        return new Pose3d(0.2157, 0.3645, 0.18, new Rotation3d(0, -getPosition(), 0));
    }

    @Log
    public Pose3d getRightBackBarComponentPose() {
        return new Pose3d(0.2157, -0.3645, 0.18, new Rotation3d(0, -getPosition(), 0));
    }

    @Log
    public boolean hasScoringElement() {
        return filter.calculate(rollersMotor.getTorqueCurrent().getValueAsDouble() > CURRENT_DETECTION_THRESHOLD);
    }

    /**
     * Sets the current of the intake's pivot
     * 
     * @param voltage The current to set the intake to
     */
    public void setPivotVoltage(double voltage) {
        pivotMotor.setControl(pivotVoltageRequest.withOutput(MathUtil.clamp(voltage, -12, 12)).withEnableFOC(true));
    }

    public Command extendPivotCommand() {
        return startEnd(() -> setPivotVoltage(Pivot.VOLTAGE), () -> setPivotVoltage(0));
    }

    public Command retractPivotCommand() {
        return startEnd(() -> setPivotVoltage(-Pivot.VOLTAGE), () -> setPivotVoltage(0));
    }

    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setPivotVoltage(speed.get() * 12), () -> setPivotVoltage(0))
                .withName("intake.setOverridenSpeedCommand");
    }

    @Override
    public Command runRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return startEnd(() -> rollersMotor.setControl(rollersVoltageRequest.withOutput(-Rollers.VOLTAGE)),
                () -> rollersMotor.stopMotor())
                .withName("intake.reverseRollers");
    }
}
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.Constants.CAN;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.subsystems.Climber.Constants.*;

import java.util.function.Supplier;

/** Subsystem which hangs robot from deep cage. */
@LoggedObject
public class Climber extends SubsystemBase {
    /** Constant values of climber subsystem. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * climber motors to be rotation away from zero point.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
        /** Current limit of climber motors. */
        public static final double CURRENT_LIMIT = 80; // TODO
        public static final double CURRENT = 80;

        /** CAN information of climber motors. */
        public static final class CAN {
            /** CAN bus climber motors are on. */
            public static final String BUS = "canivore";

            /** CAN IDs of climber motors. */
            public static final class IDs {
                /** CAN ID of climber motor A. */
                private static final int MOTOR_A = 9;
                /** CAN ID of climber motor B. */
                private static final int MOTOR_B = 10;
            }
        }
    }

    /** Climber motor A. */
    @Log
    private final TalonFX motorA = new TalonFX(CAN.IDs.MOTOR_A, CAN.BUS);
    /** Climber motor B. */
    @Log
    private final TalonFX motorB = new TalonFX(CAN.IDs.MOTOR_B, CAN.BUS);
    /**
     * Configuration object for configurations shared across both climber motors.
     */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

    /** Request object for setting motor current. */
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

    /** Initialize climber motors configurations. */
    public Climber() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;

        motorA.getConfigurator().apply(motorConfigs);
        motorB.getConfigurator().apply(motorConfigs);

        motorB.setControl(new Follower(motorA.getDeviceID(), false));

        motorA.setNeutralMode(NeutralModeValue.Brake);
        motorB.setNeutralMode(NeutralModeValue.Brake);
    }

    @Log
    public Pose3d getComponentPose() {
        return new Pose3d(0, 0.32, 0.1105, new Rotation3d(0, 0, -Math.PI / 2.0));
    }

    /**
     * Sets the voltage of both climber motors to a clamped value
     * 
     * @param current Current to set the motors to,
     */
    public void setCurrent(double current) {
        motorA.setControl(torqueCurrentRequest
                .withOutput(MathUtil.clamp(current, -CURRENT_LIMIT, CURRENT_LIMIT)));
    }

    public Command setCurrentCommand() {
        return runEnd(() -> setCurrent(CURRENT), () -> setCurrent(0)).withName("intake.setCurrentCommand");
    }

    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setCurrent(speed.get() * CURRENT), () -> setCurrent(0))
                .withName("climbr.setOverridenSpeedCommand");
    }

    /**
     * Creates a command that will make the climber motors coast instead of brake.
     * Once done, climber motors will go back to braking.
     * 
     * @return Command that causes motors to coast
     */
    public Command coastMotorsCommand() {
        return runOnce(() -> {
            motorA.stopMotor();
            motorB.stopMotor();
        }).andThen(() -> {
            motorA.setNeutralMode(NeutralModeValue.Coast);
            motorB.setNeutralMode(NeutralModeValue.Coast);
        }).finallyDo(() -> {
            motorA.setNeutralMode(NeutralModeValue.Brake);
            motorB.setNeutralMode(NeutralModeValue.Brake);
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).withName("climber.coastMotors");
    }
}

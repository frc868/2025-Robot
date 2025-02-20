package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Climber.Constants.CAN;

import static frc.robot.subsystems.Climber.Constants.*;

/** Subsystem which hangs robot from deep cage. */
public class Climber extends SubsystemBase {
    /** Constant values of climber subsystem. */
    public static final class Constants {
        /**
         * Direction of motor rotation defined as positive rotation. Defined for
         * climber motors to be rotation away from zero point.
         */
        public static final InvertedValue MOTOR_DIRECTION = InvertedValue.Clockwise_Positive; // TODO
        /** Current limit of climber motors. */
        public static final double CURRENT_LIMIT = 0; // TODO

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
    private final TalonFX motorA = new TalonFX(CAN.IDs.MOTOR_A, CAN.BUS);
    /** Climber motor B. */
    private final TalonFX motorB = new TalonFX(CAN.IDs.MOTOR_B, CAN.BUS);
    /**
     * Configuration object for configurations shared across both climber motors.
     */
    private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    /**
     * Request object for motor current
     */
    private final TorqueCurrentFOC directCurrentRequest = new TorqueCurrentFOC(0);

    /** Initialize climber motors configurations. */
    public Climber() {
        motorConfigs.MotorOutput.Inverted = MOTOR_DIRECTION;

        motorConfigs.CurrentLimits.SupplyCurrentLimit = 100;

        motorA.getConfigurator().apply(motorConfigs);
        motorB.getConfigurator().apply(motorConfigs);

        motorB.setControl(new Follower(motorA.getDeviceID(), false));

        motorA.setNeutralMode(NeutralModeValue.Brake);
        motorB.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Sets the voltage of both climber motors to a clamped value
     * 
     * @param current Current to set the motors to,
     */
    public void setCurrent(double current) {
        motorA.setControl(directCurrentRequest
                .withOutput(MathUtil.clamp(current, -CURRENT_LIMIT, CURRENT_LIMIT)));
    }

    /**
     * Sets the current of the climber to the supplied value
     * 
     * @param current
     * @return
     */
    public Command setCurrentCommand(double current) {
        return runEnd(() -> setCurrent(current), () -> setCurrent(0)).withName("climber.setCurrent");
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

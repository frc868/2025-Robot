package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;

import edu.wpi.first.wpilibj2.command.Command;
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

        public static final double VOLTAGE = 0; // TO DO

        public enum Position {
        }
    }

    public Intake() {

    }

    private TalonFX intakeMotorLift = new TalonFX(Constants.CANIDS.INTAKE_MOTOR_LIFT_CANID);
    private TalonFX intakeMotorBar = new TalonFX(Constants.CANIDS.INTAKE_MOTOR_BAR_CANID);

    @Override
    public double getPosition() {
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void resetPosition() {
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
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
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }

    @Override
    public Command runRollersCommand() {

        throw new UnsupportedOperationException("Unimplemented method 'runRollersCommand'");
    }

    @Override
    public Command reverseRollersCommand() {

        throw new UnsupportedOperationException("Unimplemented method 'reverseRollersCommand'");
    }
}

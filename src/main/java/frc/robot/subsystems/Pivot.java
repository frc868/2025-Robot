package frc.robot.subsystems;

import com.techhounds.houndutil.houndlib.subsystems.BaseSingleJointedArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** Subsystem which rotates manipulator subsystem. */
public class Pivot extends SubsystemBase implements BaseSingleJointedArm<Pivot.Constants.Position> {
    /** Constant values of pivot subsystem. */
    public static final class Constants {
        /** Positions that pivot subsystem can be in. */
        public enum Position {
            RESET_POS(0.0), // TODO get actual position
            GROUND(0.0), // TODO get actual position
            L1(0.0), // TODO get actual position
            L2(0.0), // TODO get actual position
            L3(0.0), // TODO get actual position
            L4(0.0), // TODO get actual position
            CORAL_INTAKE(0.0), // TODO get actual position
            PROCESSOR(0.0), // TODO get actual position
            REEF_ALGAE_LOW(0.0),
            REEF_ALGAE_HIGH(0.0),
            NET(0.0); // TODO get actual position

            public final double value;

            Position(double value) {
                this.value = value;
            }
        }
    }

    @Override
    public double getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void resetPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToCurrentGoalCommand'");
    }

    @Override
    public Command moveToPositionCommand(Supplier<Constants.Position> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'moveToPositionCommand'");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
                "Unimplemented method 'moveToArbitraryPositionCommand'");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'movePositionDeltaCommand'");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'holdCurrentPositionCommand'");
    }

    @Override
    public Command resetPositionCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPositionCommand'");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOverridenSpeedCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}

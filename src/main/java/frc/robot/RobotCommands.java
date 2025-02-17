package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {
    public static Command setTargetReefLevelCommand(ReefLevel reefLevel) {
        return Commands.runOnce(() -> RobotStates.targetLevel = reefLevel);
    }

    public static Command moveToScoreCommand(ReefLevel reefLevel, Elevator elevator, Pivot pivot) {
        return pivot.moveToPositionCommand(() -> reefLevel.pivotPosition)
                .until(() -> Math.abs(pivot.getPosition() - reefLevel.pivotPosition.position) <= 0.1)
                .andThen(elevator.moveToPositionCommand(() -> reefLevel.elevatorPosition));
    }

    public static Command rehomeMechanismsCommand(Elevator elevator, Pivot pivot) {
        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                .until(() -> elevator.getPosition() <= 0.1)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP));
    }

    public static Command scoreScoringElementCommand(Manipulator manipulator,
            Elevator elevator, Pivot pivot) {
        return manipulator.reverseRollersCommand().handleInterrupt(() -> rehomeMechanismsCommand(elevator, pivot));
    }
}

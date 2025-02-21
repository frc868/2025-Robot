package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Level;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {
    public static Command setModeCommand(Mode mode) {
        return Commands.runOnce(() -> RobotStates.setMode(mode)).ignoringDisable(true);
    }

    public static Command setScoringTargetLevelCommand(Supplier<Mode> mode, Level coralLevel, Level algaeLevel) {
        return Commands.runOnce(() -> RobotStates.setTargetLevel(mode.get() == Mode.CORAL ? coralLevel : algaeLevel));
    }

    public static Command moveToScoreCommand(Supplier<Level> level, Elevator elevator, Pivot pivot) {
        return pivot.moveToPositionCommand(() -> level.get().pivotPosition)
                .until(pivot::atGoal)
                .andThen(elevator.moveToPositionCommand(() -> level.get().elevatorPosition))
                .until(elevator::atGoal);
    }

    public static Command moveToScoreCommandComp(Supplier<Level> reefLevel, Elevator elevator) {
        return elevator.moveToPositionCommand(() -> reefLevel.get().elevatorPosition)
                .until(() -> {
                    double elevatorCurrentPosition = elevator.getPosition();
                    double elevatorTargetPosition = reefLevel.get().elevatorPosition.position;
                    boolean isElevatorAtTarget = Math
                            .abs(elevatorCurrentPosition - elevatorTargetPosition) <= 0.05;
                    return isElevatorAtTarget;
                })
                .andThen(elevator.holdCurrentPositionCommand());
    }

    public static Command rehomeMechanismsCommand(Elevator elevator, Pivot pivot,
            Manipulator manipulator) {
        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                .until(elevator::atGoal)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP));
    }

    public static Command rehomeMechanismsCommandComp(Elevator elevator, Pivot pivot, Manipulator manipulator) {
        return new SequentialCommandGroup(
                pivot.moveToPositionCommand(() -> Pivot.Constants.Position.PAST_ELEVATOR)
                        .until(() -> pivot.getPosition() <= 0.05),
                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                        .until(() -> elevator.getPosition() <= 0.05),
                pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP)
                        .until(() -> pivot.getPosition() <= 0.05),
                manipulator.runRollersCommand());
    }

    public static Command scoreScoringElementCommand(Manipulator manipulator,
            Elevator elevator, Pivot pivot) {
        return manipulator.reverseRollersCommand()
                .finallyDo(() -> rehomeMechanismsCommand(elevator, pivot, manipulator));
    }
}
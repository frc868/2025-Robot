package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ReefLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {
    public static Command setTargetReefLevelCommand(ReefLevel reefLevel) {
        return Commands.runOnce(() -> {
            RobotStates.setTargetLevel(reefLevel);
            System.out.println("Current target level:" + reefLevel);
            System.out.println("Current robot state target level: " + RobotStates.getTargetLevel());
        });
    }

    public static Command moveToScoreCommand(Supplier<ReefLevel> reefLevel, Elevator elevator, Pivot pivot) {
        return pivot.moveToPositionCommand(() -> reefLevel.get().pivotPosition)
                .until(() -> {
                    double currentPosition = pivot.getPosition();
                    double targetPosition = reefLevel.get().pivotPosition.position;
                    boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.05;
                    System.out.println("Pivot position: " + currentPosition + ", Target: " + targetPosition
                            + ", At Target: " + isAtTarget);
                    return isAtTarget;
                })
                // .andThen(pivot.holdCurrentPositionCommand())
                .andThen(elevator.moveToPositionCommand(() -> reefLevel.get().elevatorPosition));
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

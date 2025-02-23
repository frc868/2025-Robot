package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Level;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.*;

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {
    public static Command setModeCommand(Mode mode) {
        return Commands.runOnce(() -> RobotStates.setMode(mode)).ignoringDisable(true);
    }

    public static Command setScoringTargetLevelCommand(Supplier<Mode> mode, Level coralLevel, Level algaeLevel) {
        return Commands.runOnce(
                () -> RobotStates.setTargetLevel(
                        mode.get() == Mode.CORAL || mode.get() == null ? coralLevel
                                : algaeLevel));
        // .andThen(() -> {
        // System.out.println(mode.get().toString());
        // });
    }

    public static Command moveToTargetLevelCommand(Supplier<Level> reefLevel, Elevator elevator, Pivot pivot) {
        return pivot.moveToPositionCommand(() -> reefLevel.get().pivotPosition)
                .until(() -> {
                    double currentPosition = pivot.getPosition();
                    double targetPosition = reefLevel.get().pivotPosition.position;
                    boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.01;
                    // System.out.println("Pivot position: " + currentPosition + ", Target: "
                    // + targetPosition
                    // + ", At Target: " + isAtTarget);
                    return isAtTarget;
                })
                .andThen(pivot.holdCurrentPositionCommand())
                .andThen(elevator.moveToPositionCommand(() -> reefLevel.get().elevatorPosition))
                .until(() -> {
                    double elevatorCurrentPosition = elevator.getPosition();
                    double elevatorTargetPosition = reefLevel.get().elevatorPosition.position;
                    boolean isElevatorAtTarget = Math
                            .abs(elevatorCurrentPosition - elevatorTargetPosition) <= 0.01;
                    // System.out.println(
                    // "Elevator position: " + elevatorCurrentPosition + ", Target: "
                    // + elevatorTargetPosition
                    // + ", At Target: " + isElevatorAtTarget);
                    return isElevatorAtTarget;
                })
                .andThen(elevator.holdCurrentPositionCommand());
    }

    public static Command moveToTargetLevelIntakeCommand(Supplier<Level> reefLevel, Elevator elevator,
            Pivot pivot) {

        return pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SAFE)
                .until(() -> {
                    double currentSafePosition = pivot.getPosition();
                    double targetSafePosition = Pivot.Constants.Position.SAFE.position;
                    boolean isAtSafeTarget = Math
                            .abs(currentSafePosition - targetSafePosition) <= 0.01;
                    return isAtSafeTarget;
                })
                .andThen(pivot.holdCurrentPositionCommand())
                .andThen(elevator.moveToPositionCommand(() -> Elevator.Constants.Position.GROUND_ALGAE))
                .until(() -> {
                    double elevatorCurrentPosition = elevator.getPosition();
                    double elevatorTargetPosition = Elevator.Constants.Position.GROUND_ALGAE.position;
                    boolean isElevatorAtTarget = Math
                            .abs(elevatorCurrentPosition - elevatorTargetPosition) <= 0.01;
                    return isElevatorAtTarget;
                })
                .andThen(elevator.holdCurrentPositionCommand())
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.GROUND_ALGAE))
                .until(() -> {
                    double currentPosition = pivot.getPosition();
                    double targetPosition = Pivot.Constants.Position.GROUND_ALGAE.position;
                    boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.01;
                    return isAtTarget;
                });
    }

    // public static Command moveToTargetLevelCommand(Supplier<Level> level,
    // Elevator elevator, Pivot pivot) {
    // return pivot.moveToPositionCommand(() -> level.get().pivotPosition)
    // .until(pivot::atGoal)
    // .andThen(elevator.moveToPositionCommand(() -> level.get().elevatorPosition))
    // .until(elevator::atGoal);
    // }

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

    // public static Command rehomeMechanismsCommand(Elevator elevator, Pivot pivot,
    // Manipulator manipulator) {
    // return pivot.moveToPositionCommand(() ->
    // Pivot.Constants.Position.PAST_ELEVATOR)
    // .until(pivot::atGoal)
    // .andThen(elevator.moveToPositionCommand(() ->
    // Elevator.Constants.Position.HARD_STOP))
    // .until(elevator::atGoal)
    // .andThen(pivot.moveToPositionCommand(() ->
    // Pivot.Constants.Position.HARD_STOP));
    // }

    public static Command rehomeMechanismsCommand(Elevator elevator, Pivot pivot,
            Manipulator manipulator, Intake intake) {
        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                .until(() -> elevator.getPosition() <= 0.05)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP)
                        .until(() -> pivot.getPosition() <= 0.05),
                        // intake.retractPivotCommand(),
                        manipulator.intakeScoringElementCommand());
    }

    public static Command rehomeMechanismsCommandComp(Elevator elevator, Pivot pivot, Manipulator manipulator,
            Intake intake) {
        return new SequentialCommandGroup(
                pivot.moveToPositionCommand(() -> Pivot.Constants.Position.PAST_ELEVATOR)
                        .until(() -> pivot.getPosition() <= 0.05),
                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                        .until(() -> elevator.getPosition() <= 0.05),
                pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP)
                        .until(() -> pivot.getPosition() <= 0.05),
                intake.retractPivotCommand(),
                manipulator.runRollersCommand());
    }

    public static Command scoreScoringElementCommand(Manipulator manipulator,
            Elevator elevator, Pivot pivot, Intake intake) {
        return manipulator.reverseRollersCommand()
                .finallyDo(() -> rehomeMechanismsCommand(elevator, pivot, manipulator, intake));
    }

    public static Command intakeGroundAlgaeCommand(Elevator elevator, Pivot pivot, Manipulator manipulator,
            Intake intake) {
        return intake.extendPivotCommand().withTimeout(0.5)
                .andThen(intake.intakeGroundAlgaeCommand())
                // .withTimeout(0.5)
                .andThen(Commands.waitSeconds(1))
                .alongWith(RobotCommands.moveToTargetLevelIntakeCommand(() -> Level.GROUND,
                        elevator, pivot));
    }
}
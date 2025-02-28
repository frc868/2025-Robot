package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Level;
import frc.robot.subsystems.*;

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {
    public static Command setScoringTargetLevelCommand(Level level) {
        return Commands.runOnce(() -> RobotStates.setTargetLevel(level));
    }

    public static Command moveToTargetLevelCommand(Supplier<Level> reefLevel, Elevator elevator, Pivot pivot) {
        return pivot.moveToPositionCommand(() -> reefLevel.get().pivotPosition)
                .until(() -> {
                    double currentPosition = pivot.getPosition();
                    double targetPosition = reefLevel.get().pivotPosition.position;
                    boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.01;
                    return isAtTarget;
                })
                .andThen(pivot.holdCurrentPositionCommand())
                .andThen(elevator.moveToPositionCommand(() -> reefLevel.get().elevatorPosition))
                .until(() -> {
                    double elevatorCurrentPosition = elevator.getPosition();
                    double elevatorTargetPosition = reefLevel.get().elevatorPosition.position;
                    boolean isElevatorAtTarget = Math
                            .abs(elevatorCurrentPosition - elevatorTargetPosition) <= 0.01;
                    return isElevatorAtTarget;
                })
                .andThen(elevator.holdCurrentPositionCommand());
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
            Manipulator manipulator, Intake intake) {
        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                .until(() -> elevator.getPosition() <= 0.01)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP)
                        .until(() -> pivot.getPosition() <= 0.01),
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

    // Ground Algae
    public static Command intakeGroundAlgaeCommand(Elevator elevator, Pivot pivot, Manipulator manipulator,
            Intake intake) {
        return Commands.sequence(
                intake.extendPivotCommand().withTimeout(1),

                RobotCommands.moveToTargetLevelIntakeCommand(() -> Level.GROUND, elevator, pivot),

                Commands.parallel(
                        manipulator.runRollersCommand(),
                        intake.runRollersCommand().until(manipulator::hasScoringElement)
                                .andThen(pivot.moveToPositionCommand(
                                        () -> Pivot.Constants.Position.PAST_ELEVATOR)
                                        .until(() -> {
                                            double currentPosition = pivot
                                                    .getPosition();
                                            double targetPosition = Pivot.Constants.Position.PAST_ELEVATOR.position;
                                            boolean isAtTarget = Math
                                                    .abs(currentPosition
                                                            - targetPosition) <= 0.01;
                                            return isAtTarget;
                                        }))
                                .andThen(intake.retractPivotCommand().withTimeout(2)))

        );
    }

    public static Command rehomeIntakeMechanismsCommand(Elevator elevator, Pivot pivot,
            Manipulator manipulator, Intake intake) {
        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.HARD_STOP)
                .until(() -> elevator.getPosition() <= 0.01)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.HARD_STOP)
                        .until(() -> pivot.getPosition() <= 0.01),
                        intake.reverseRollersCommand().withTimeout(1.5),
                        intake.retractPivotCommand().withTimeout(1.5),
                        manipulator.intakeScoringElementCommand());
    }

    // Climb
    public static Command climbCommand(Pivot pivot, Intake intake, Climber climber, Manipulator manipulator) {
        return intake.retractPivotCommand().withTimeout(1.5)
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.CLIMB_RETRACT))
                .until(() -> {
                    double currentSafePosition = pivot.getPosition();
                    double targetSafePosition = Pivot.Constants.Position.CLIMB_RETRACT.position;
                    boolean isAtSafeTarget = Math
                            .abs(currentSafePosition - targetSafePosition) <= 0.01;
                    return isAtSafeTarget;
                })
                .andThen(manipulator.stopRollersCommand())
                .andThen(pivot.holdCurrentPositionCommand());

        // .alongWith(climber.setCurrentCommand());
    }

    // Net
    public static Command moveToTargetLevelNetCommand(Supplier<Level> reefLevel, Elevator elevator,
            Pivot pivot) {

        return elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L4_NET)
                .until(() -> {
                    double elevatorCurrentPosition = elevator.getPosition();
                    double elevatorTargetPosition = Elevator.Constants.Position.L4_NET.position;
                    boolean isElevatorAtTarget = Math
                            .abs(elevatorCurrentPosition - elevatorTargetPosition) <= 0.005;
                    return isElevatorAtTarget;
                })
                .andThen(elevator.holdCurrentPositionCommand())
                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.NET))
                .until(() -> {
                    double currentPosition = pivot.getPosition();
                    double targetPosition = Pivot.Constants.Position.NET.position;
                    boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.01;
                    return isAtTarget;
                })
                .andThen(pivot.holdCurrentPositionCommand());
    }
}
package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

        // public static Command intakeGroundAlgaeCommand(Elevator elevator, Pivot
        // pivot, Manipulator manipulator,
        // Intake intake) {
        // return intake.extendPivotCommand().withTimeout(1)
        // // .withTimeout(0.5)
        // .andThen(Commands.waitSeconds(1))
        // .alongWith(RobotCommands.moveToTargetLevelIntakeCommand(() -> Level.GROUND,
        // elevator, pivot))
        // .andThen(intake.runRollersCommand().until(manipulator::hasScoringElement))
        // .andThen(RobotCommands.moveToTargetLevelCommand(() -> Level.L2, elevator,
        // pivot));
        // }

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
                                        System.out.println(
                                                        "Elevator position: " + elevatorCurrentPosition + ", Target: "
                                                                        + elevatorTargetPosition
                                                                        + ", At Target: " + isElevatorAtTarget);
                                        return isElevatorAtTarget;
                                })
                                .andThen(elevator.holdCurrentPositionCommand())
                                .andThen(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.NET))
                                .until(() -> {
                                        double currentPosition = pivot.getPosition();
                                        double targetPosition = Pivot.Constants.Position.NET.position;
                                        boolean isAtTarget = Math.abs(currentPosition - targetPosition) <= 0.01;
                                        System.out.println("Pivot position: " + currentPosition + ", Target: "
                                                        + targetPosition
                                                        + ", At Target: " + isAtTarget);
                                        return isAtTarget;
                                })
                                .andThen(pivot.holdCurrentPositionCommand());
        }
}
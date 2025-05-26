package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;
import com.techhounds.houndutil.houndlib.BooleanContainer;
import com.techhounds.houndutil.houndlib.DoubleContainer;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Elevator.ElevatorProfileParams;
import frc.robot.FieldConstants.Reef.ReefBranch;
import frc.robot.FieldConstants.Reef.ReefLevel;
import frc.robot.FieldConstants.Reef.ReefSide;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

import static frc.robot.Constants.Auto.*;

public class Autos {
    public static AutoRoutine wheelRadiusCharacterization(Drivetrain drivetrain) {
        // wheel radius (meters) = gyro delta (radians) * drive base radius (meters) /
        // wheel position delta (radians)

        DoubleContainer initialGyroRotation = new DoubleContainer(0);
        DoubleContainer initialWheelPosition = new DoubleContainer(0);
        Command command = Commands.sequence(
                drivetrain.teleopDriveCommand(() -> 0, () -> 0, () -> 0.5).alongWith(
                        Commands.waitSeconds(1).andThen(
                                Commands.runOnce(() -> {
                                    initialGyroRotation.value = drivetrain.getRawYaw().in(Radians);
                                    initialWheelPosition.value = Rotations
                                            .of(drivetrain.getModulePositions()[0].distanceMeters
                                                    / Constants.Drivetrain.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE)
                                            .in(Radians);
                                })))
                        .withTimeout(10),

                Commands.waitSeconds(0.5),
                Commands.runOnce(() -> {
                    System.out.println("################################################");
                    System.out.println("######## Wheel Characterization Results ########");
                    System.out.println("################################################");

                    double gyroDelta = drivetrain.getRawYaw().in(Radians) - initialGyroRotation.value;
                    double wheelPositionDelta = Math.abs(Rotations.of(drivetrain.getModulePositions()[0].distanceMeters
                            / Constants.Drivetrain.SWERVE_CONSTANTS.WHEEL_CIRCUMFERENCE).in(Radians)
                            - initialWheelPosition.value);

                    double wheelRadius = (gyroDelta * Constants.Drivetrain.DRIVE_BASE_RADIUS_METERS)
                            / wheelPositionDelta;

                    System.out.println("Gyro Delta (rad): " + gyroDelta);
                    System.out.println("Wheel Position Delta (rad): " + wheelPositionDelta);
                    System.out.println("Wheel Radius (m): " + wheelRadius);
                }));

        return new AutoRoutine("WheelRadius", command, List.of(), Pose2d.kZero);
    }

    public static AutoRoutine FDC(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToF = PathPlannerPath.fromChoreoTrajectory("FDCB", 0);
        PathPlannerPath pathFToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 1);
        PathPlannerPath pathIntakeToD = PathPlannerPath.fromChoreoTrajectory("FDCB", 2);
        PathPlannerPath pathDToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 3);
        PathPlannerPath pathIntakeToC = PathPlannerPath.fromChoreoTrajectory("FDCB", 4);
        PathPlannerPath pathCToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 5);
        PathPlannerPath pathIntakeToB = PathPlannerPath.fromChoreoTrajectory("FDCB", 6);
        Pose2d startingPose = pathStartToF.getStartingHolonomicPose().get();

        Command command = Commands.sequence(
                Commands.waitSeconds(0.2).andThen(drivetrain.lockToClosestBranchAutoCommand(ReefBranch.F))
                        .alongWith(superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                superstructure.scoreCoralCommand(),

                Commands.waitSeconds(0.2).andThen(drivetrain.followPathCommand(pathFToIntake))
                        .alongWith(superstructure.stowAfterScoreCommand()),
                Commands.waitUntil(superstructure.manipulator.hasCoral).withTimeout(1.5),
                drivetrain.lockToClosestBranchAutoCommand(ReefBranch.D)
                        .alongWith(superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                superstructure.scoreCoralCommand(),

                Commands.waitSeconds(0.2).andThen(drivetrain.followPathCommand(pathDToIntake))
                        .alongWith(superstructure.stowAfterScoreCommand()),
                Commands.waitUntil(superstructure.manipulator.hasCoral).withTimeout(1.5),
                drivetrain.lockToClosestBranchAutoCommand(ReefBranch.C)
                        .alongWith(superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                superstructure.scoreCoralCommand(),

                Commands.waitSeconds(0.2).andThen(drivetrain.followPathCommand(pathCToIntake))
                        .alongWith(superstructure.stowAfterScoreCommand()));

        // Commands.waitSeconds(1).andThen(drivetrain.followPathCommand(pathCToIntake))
        // .alongWith(superstructure.stowAfterScoreCommand()),
        // // Commands.waitSeconds(0.4),
        // Commands.waitUntil(superstructure.manipulator.hasCoral),
        // drivetrain.followPathCommand(pathIntakeToB)
        // .alongWith(Commands.waitSeconds(1.4)
        // .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4, elevator,
        // arm))),
        // superstructure.scoreCoralCommand(manipulator));

        return new AutoRoutine("FDC", command, List.of(pathStartToF, pathFToIntake, pathIntakeToD, pathDToIntake,
                pathIntakeToC, pathCToIntake, pathIntakeToB), startingPose);
    }

    public static AutoRoutine FDCB(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToF = PathPlannerPath.fromChoreoTrajectory("FDCB", 0);
        PathPlannerPath pathFToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 1);
        PathPlannerPath pathIntakeToD = PathPlannerPath.fromChoreoTrajectory("FDCB", 2);
        PathPlannerPath pathDToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 3);
        PathPlannerPath pathIntakeToC = PathPlannerPath.fromChoreoTrajectory("FDCB", 4);
        PathPlannerPath pathCToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 5);
        PathPlannerPath pathIntakeToB = PathPlannerPath.fromChoreoTrajectory("FDCB", 6);
        Pose2d startingPose = pathStartToF.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseTallCommand(ReefBranch.F,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                drivetrain.followPathCommand(pathFToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.D,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                                .and(() -> drivetrain.getCurrentBranchDistance() < 3.3)),
                                        Commands.runOnce(() -> startedScoring.value = true),
                                        superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        // stop if the timer is above 4 seconds, we haven't started scoring yet, and we
                        // don't have a game piece
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathDToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.C,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathCToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPause4thCommand(ReefBranch.B,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),

                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.8))
                                .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-2.0, 0, 0), DriveMode.ROBOT_RELATIVE),
                        drivetrain)
                        .withTimeout(0.5))
                .finallyDo(() -> superstructure.elevator.setElevatorProfileParams(ElevatorProfileParams.MID));

        return new AutoRoutine("FDCB", command, List.of(pathStartToF, pathFToIntake, pathIntakeToD, pathDToIntake,
                pathIntakeToC, pathCToIntake, pathIntakeToB), startingPose);
    }

    public static AutoRoutine FDCE(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToF = PathPlannerPath.fromChoreoTrajectory("FDCB", 0);
        PathPlannerPath pathFToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 1);
        PathPlannerPath pathIntakeToD = PathPlannerPath.fromChoreoTrajectory("FDCB", 2);
        PathPlannerPath pathDToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 3);
        PathPlannerPath pathIntakeToC = PathPlannerPath.fromChoreoTrajectory("FDCB", 4);
        PathPlannerPath pathCToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 5);
        PathPlannerPath pathIntakeToB = PathPlannerPath.fromChoreoTrajectory("FDCB", 6);
        Pose2d startingPose = pathStartToF.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseTallCommand(ReefBranch.F,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                drivetrain.followPathCommand(pathFToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.D,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                                .and(() -> drivetrain.getCurrentBranchDistance() < 3.3)),
                                        Commands.runOnce(() -> startedScoring.value = true),
                                        superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        // stop if the timer is above 4 seconds, we haven't started scoring yet, and we
                        // don't have a game piece
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathDToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.C,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathCToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPause4thCommand(ReefBranch.E,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),
                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-2.5, 0, 0), DriveMode.ROBOT_RELATIVE),
                        drivetrain).withTimeout(0.5))
                .finallyDo(() -> superstructure.elevator.setElevatorProfileParams(ElevatorProfileParams.MID));

        return new AutoRoutine("FDCE", command, List.of(pathStartToF, pathFToIntake, pathIntakeToD, pathDToIntake,
                pathIntakeToC, pathCToIntake, pathIntakeToB), startingPose);
    }

    public static AutoRoutine FDCBExperimental(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToF = PathPlannerPath.fromChoreoTrajectory("FDCB", 0);
        PathPlannerPath pathFToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 1);
        PathPlannerPath pathIntakeToD = PathPlannerPath.fromChoreoTrajectory("FDCB", 2);
        PathPlannerPath pathDToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 3);
        PathPlannerPath pathIntakeToC = PathPlannerPath.fromChoreoTrajectory("FDCB", 4);
        PathPlannerPath pathCToIntake = PathPlannerPath.fromChoreoTrajectory("FDCB", 5);
        PathPlannerPath pathIntakeToB = PathPlannerPath.fromChoreoTrajectory("FDCB", 6);
        Pose2d startingPose = pathStartToF.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseFastCommand(ReefBranch.F,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.driveToPoseCommand(() -> RIGHT_INTAKING_LOCATION)
                        .until(() -> drivetrain.isInGoalPosition(RIGHT_INTAKING_LOCATION))
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.D,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.8)),
                                        Commands.runOnce(() -> startedScoring.value = true),
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        // stop if the timer is above 4 seconds, we haven't started scoring yet, and we
                        // don't have a game piece
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.driveToPoseCommand(() -> RIGHT_INTAKING_LOCATION)
                        .until(() -> drivetrain.isInGoalPosition(RIGHT_INTAKING_LOCATION))
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.C,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 2.8))
                                        .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.driveToPoseCommand(() -> RIGHT_INTAKING_LOCATION)
                        .until(() -> drivetrain.isInGoalPosition(RIGHT_INTAKING_LOCATION))
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.B,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),
                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.2))
                                .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                drivetrain.teleopDriveCommand(() -> -0.5, () -> 0, () -> 0).withTimeout(0.5));

        return new AutoRoutine("FDCBExperimental", command,
                List.of(pathStartToF, pathFToIntake, pathIntakeToD, pathDToIntake,
                        pathIntakeToC, pathCToIntake, pathIntakeToB),
                startingPose);
    }

    public static AutoRoutine FBDC(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToF = PathPlannerPath.fromChoreoTrajectory("FBDC", 0);
        PathPlannerPath pathFToIntake = PathPlannerPath.fromChoreoTrajectory("FBDC", 1);
        PathPlannerPath pathIntakeToB = PathPlannerPath.fromChoreoTrajectory("FBDC", 2);
        PathPlannerPath pathBToIntake = PathPlannerPath.fromChoreoTrajectory("FBDC", 3);
        PathPlannerPath pathIntakeToD = PathPlannerPath.fromChoreoTrajectory("FBDC", 4);
        PathPlannerPath pathDToIntake = PathPlannerPath.fromChoreoTrajectory("FBDC", 5);
        PathPlannerPath pathIntakeToC = PathPlannerPath.fromChoreoTrajectory("FBDC", 6);
        PathPlannerPath pathCToIntake = PathPlannerPath.fromChoreoTrajectory("FBDC", 7);
        Pose2d startingPose = pathStartToF.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseFastCommand(ReefBranch.F,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathFToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPause4thCommand(ReefBranch.B,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),
                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.2))
                                .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                drivetrain.followPathCommand(pathBToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.D,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathDToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.C,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreCommand(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathCToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()));

        return new AutoRoutine("FBDC_Different", command,
                List.of(pathStartToF, pathFToIntake, pathIntakeToD, pathDToIntake,
                        pathIntakeToC, pathDToIntake, pathIntakeToB, pathCToIntake),
                startingPose);
    }

    public static AutoRoutine HAlgae(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseFastCommand(ReefBranch.H,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.driveToPoseCommand(() -> drivetrain.getCurrentBranchTargetPose()
                        .plus(new Transform2d(-0.6, 0, Rotation2d.kZero))).withTimeout(0.5),

                Commands.waitUntil(superstructure.manipulator.hasAlgae).andThen(Commands.waitSeconds(0.25))
                        .deadlineFor(superstructure.algaeReefIntakeAutoCommand(ReefLevel.L2)
                                .alongWith(drivetrain.lockToClosestReefAlgaeAutoCommand(ReefSide.FOUR,
                                        superstructure.manipulator.hasAlgae))),
                superstructure.moveToAutoNetScoreCommand(drivetrain)
                        .alongWith(drivetrain.lockToNetScoreAutoCommand(new Transform2d(0, -0.4, Rotation2d.kZero))
                                .until(drivetrain::isInNetScorePosition)),
                superstructure.finishAutoNetScoreAutoCommand(),

                Commands.waitUntil(superstructure.manipulator.hasAlgae).andThen(Commands.waitSeconds(0.25))
                        .deadlineFor(superstructure.algaeReefIntakeAutoCommand(ReefLevel.L3)
                                .alongWith(drivetrain.lockToClosestReefAlgaeAutoCommand(ReefSide.FIVE,
                                        superstructure.manipulator.hasAlgae))),
                superstructure.moveToAutoNetScoreCommand(drivetrain)
                        .alongWith(drivetrain.lockToNetScoreAutoCommand(new Transform2d(0, -0.2, Rotation2d.kZero))
                                .until(drivetrain::isInNetScorePosition)),
                superstructure.finishAutoNetScoreAutoCommand(),

                Commands.waitUntil(superstructure.manipulator.hasAlgae).andThen(Commands.waitSeconds(0.25))
                        .deadlineFor(superstructure.algaeReefIntakeAutoCommand(ReefLevel.L3)
                                .alongWith(drivetrain.lockToClosestReefAlgaeAutoCommand(ReefSide.THREE,
                                        superstructure.manipulator.hasAlgae))),
                superstructure.moveToAutoNetScoreCommand(drivetrain)
                        .alongWith(drivetrain.lockToNetScoreAutoThirdCommand()
                                .until(drivetrain::isInNetScorePosition)),
                superstructure.finishAutoNetScoreAutoCommand(),

                drivetrain.teleopDriveCommand(() -> -0.5, () -> 0, () -> 0).withTimeout(0.5))
                .finallyDo(() -> superstructure.elevator.setElevatorProfileParams(ElevatorProfileParams.MID));

        return new AutoRoutine("HAlgae", command, List.of(), new Pose2d(7.143, 4.153, Rotation2d.k180deg));
    }

    public static AutoRoutine IKLA(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToI = PathPlannerPath.fromChoreoTrajectory("IKLA", 0);
        PathPlannerPath pathIToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 1);
        PathPlannerPath pathIntakeToK = PathPlannerPath.fromChoreoTrajectory("IKLA", 2);
        PathPlannerPath pathKToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 3);
        PathPlannerPath pathIntakeToL = PathPlannerPath.fromChoreoTrajectory("IKLA", 4);
        PathPlannerPath pathLToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 5);
        PathPlannerPath pathIntakeToA = PathPlannerPath.fromChoreoTrajectory("IKLA", 6);
        Pose2d startingPose = pathStartToI.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseTallCommand(ReefBranch.I,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                drivetrain.followPathCommand(pathIToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.K,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                                .and(() -> drivetrain.getCurrentBranchDistance() < 3.3)),
                                        Commands.runOnce(() -> startedScoring.value = true),
                                        superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        // stop if the timer is above 4 seconds, we haven't started scoring yet, and we
                        // don't have a game piece
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathKToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.L,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathLToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPause4thCommand(ReefBranch.A,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),
                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.8))
                                .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-2.5, 0, 0), DriveMode.ROBOT_RELATIVE),
                        drivetrain).withTimeout(0.5))
                .finallyDo(() -> superstructure.elevator.setElevatorProfileParams(ElevatorProfileParams.MID));

        return new AutoRoutine("IKLA", command, List.of(pathStartToI, pathIToIntake, pathIntakeToK, pathKToIntake,
                pathIntakeToL, pathLToIntake, pathIntakeToA), startingPose);
    }

    public static AutoRoutine IKLJ(Drivetrain drivetrain, Superstructure superstructure)
            throws IOException, ParseException {
        PathPlannerPath pathStartToI = PathPlannerPath.fromChoreoTrajectory("IKLA", 0);
        PathPlannerPath pathIToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 1);
        PathPlannerPath pathIntakeToK = PathPlannerPath.fromChoreoTrajectory("IKLA", 2);
        PathPlannerPath pathKToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 3);
        PathPlannerPath pathIntakeToL = PathPlannerPath.fromChoreoTrajectory("IKLA", 4);
        PathPlannerPath pathLToIntake = PathPlannerPath.fromChoreoTrajectory("IKLA", 5);
        PathPlannerPath pathIntakeToA = PathPlannerPath.fromChoreoTrajectory("IKLA", 6);
        Pose2d startingPose = pathStartToI.getStartingHolonomicPose().get();

        BooleanContainer startedScoring = new BooleanContainer(false);
        Timer timer = new Timer();

        Command command = Commands.sequence(
                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        drivetrain
                                .lockToClosestBranchAutoPauseTallCommand(ReefBranch.I,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition))
                                .alongWith(
                                        superstructure.prepareCoralScoreCommand(ReefLevel.L4)),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                drivetrain.followPathCommand(pathIToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.K,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                                .and(() -> drivetrain.getCurrentBranchDistance() < 3.3)),
                                        Commands.runOnce(() -> startedScoring.value = true),
                                        superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        // stop if the timer is above 4 seconds, we haven't started scoring yet, and we
                        // don't have a game piece
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathKToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                Commands.waitSeconds(0.2),
                Commands.runOnce(timer::reset),
                Commands.sequence(
                        Commands.runOnce(timer::start),
                        Commands.parallel(
                                drivetrain.lockToClosestBranchAutoPauseFastCommand(ReefBranch.L,
                                        superstructure.manipulator.hasGamePiece
                                                .and(superstructure.isInAutoScoringPosition)),
                                Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                        .and(() -> drivetrain.getCurrentBranchDistance() < 3.3))
                                        .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                        superstructure.scoreCoralCommand())
                        .until(() -> (timer.get() > 4 && startedScoring.value == false
                                && !superstructure.manipulator.hasGamePiece.getAsBoolean()))
                        .finallyDo(() -> startedScoring.value = false),

                drivetrain.followPathCommand(pathLToIntake)
                        .alongWith(superstructure.stowAfterScoreCommand()),

                superstructure.elevator.setElevatorProfileParamsCommand(ElevatorProfileParams.SLOW),

                Commands.waitSeconds(0.2),
                Commands.parallel(
                        drivetrain.lockToClosestBranchAutoPause4thCommand(ReefBranch.J,
                                superstructure.manipulator.hasGamePiece.and(superstructure.isInAutoScoringPosition)),
                        Commands.waitUntil(superstructure.manipulator.hasGamePiece
                                .and(() -> drivetrain.getCurrentBranchDistance() < 2.8))
                                .andThen(superstructure.prepareCoralScoreFastL4Command(ReefLevel.L4))),
                superstructure.scoreCoralCommand(),

                Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-2.5, 0, 0), DriveMode.ROBOT_RELATIVE),
                        drivetrain).withTimeout(0.5))
                .finallyDo(() -> superstructure.elevator.setElevatorProfileParams(ElevatorProfileParams.MID));

        return new AutoRoutine("IKLJ", command, List.of(pathStartToI, pathIToIntake, pathIntakeToK, pathKToIntake,
                pathIntakeToL, pathLToIntake, pathIntakeToA), startingPose);
    }

}
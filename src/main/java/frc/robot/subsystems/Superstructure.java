package frc.robot.subsystems;

import java.util.Map;
import java.util.Optional;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive.DriveMode;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.FieldConstants.Reef.ReefLevel;

@LoggedObject
public class Superstructure extends SubsystemBase {
    public final Elevator elevator;
    public final Arm arm;
    public final Manipulator manipulator;
    public final Climber climber;
    public final LEDs leds;

    @Log
    public final Trigger isInAutoScoringPosition;

    @Log
    public final Trigger isInAlgaeReefPickupPosition;
    @Log
    public final Trigger isReadyForNetScore;

    // blanket score readiness
    @Log
    public final Trigger isReadyToScore;

    private boolean isInAlgaeReefPickupPositionBool;
    private boolean isReadyForNetScoreBool;
    @Log
    private Optional<ReefLevel> reefSetpoint = Optional.empty();

    public Superstructure(Drivetrain drivetrain, Elevator elevator, Arm arm, Manipulator manipulator, Climber climber,
            LEDs leds) {
        this.elevator = elevator;
        this.arm = arm;
        this.manipulator = manipulator;
        this.climber = climber;
        this.leds = leds;

        this.isInAutoScoringPosition = new Trigger(() -> elevator.getPosition() > 0.75);
        this.isInAlgaeReefPickupPosition = new Trigger(() -> isInAlgaeReefPickupPositionBool);
        this.isReadyForNetScore = new Trigger(() -> isReadyForNetScoreBool);

        this.isReadyToScore = new Trigger(() -> {
            if (isInAlgaeReefPickupPosition.getAsBoolean()) {
                return drivetrain.isInAlgaeReefPickupPosition();
            }

            if (isReadyForNetScore.getAsBoolean()) {
                return drivetrain.isInNetScorePosition() && elevator.getPosition() > 1.45;
            }

            var setpoint = getReefSetpoint().orElse(null);
            if (setpoint == null)
                return false;

            return switch (setpoint) {
                case L1 -> drivetrain.isInTroughPosition();
                case L2, L3 -> drivetrain.isInL2L3ScoringPosition();
                case L4 -> drivetrain.isInL4ScoringPosition();
            };
        });
    }

    public Command useRequirement() {
        return runOnce(() -> {
        });
    }

    public Optional<ReefLevel> getReefSetpoint() {
        return reefSetpoint;
    }

    public Command setReefSetpointCommand(ReefLevel reefLevel) {
        return Commands.runOnce(() -> reefSetpoint = Optional.of(reefLevel)).withName("elevator.setReefSetpoint");
    }

    public Command clearReefSetpointCommand() {
        return Commands.runOnce(() -> reefSetpoint = Optional.empty()).withName("elevator.clearReefSetpoint");
    }

    public Command stowCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> isReadyForNetScoreBool = false),
                clearReefSetpointCommand(),
                Commands.either(
                        Commands.parallel(
                                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                                Commands.waitUntil(() -> elevator.getPosition() < 0.3)
                                        .andThen(arm.moveToPositionCommand(() -> ArmPosition.CORAL_INTAKE).asProxy())),
                        Commands.parallel(
                                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                                Commands.waitUntil(() -> elevator.getPosition() < 0.1)
                                        .andThen(arm.moveToPositionCommand(() -> ArmPosition.CORAL_INTAKE).asProxy())),
                        () -> elevator.getPosition() > 0.4))
                .andThen(useRequirement()).withName("RobotCommands.stow");
    }

    public Command stowAfterScoreCommand() {
        return Commands.runOnce(() -> isReadyForNetScoreBool = false).andThen(
                Commands.parallel(
                        Commands.deadline(
                                Commands.waitUntil(() -> elevator.getPosition() < 0.1),
                                arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE).asProxy())
                                .andThen(arm.moveToPositionCommand(() -> ArmPosition.CORAL_INTAKE).asProxy()),
                        Commands.waitUntil(() -> arm.getPosition() > 0).andThen(
                                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy())))
                .andThen(useRequirement()).withName("RobotCommands.stowAfterScore");
    }

    public Command algaeGroundIntakeCommand(Drivetrain drivetrain) {
        return Commands.sequence(
                Commands.runOnce(() -> isReadyForNetScoreBool = true),
                Commands.waitUntil(manipulator.hasAlgae).deadlineFor(
                        Commands.either(
                                Commands.parallel(
                                        manipulator.groundIntakeAlgaeRollersCommand().asProxy(),
                                        arm.moveToPositionCommand(() -> ArmPosition.ALGAE_GROUND_INTAKE).asProxy(),
                                        Commands.waitUntil(() -> arm.getPosition() < 1.5)
                                                .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM)
                                                        .asProxy())),
                                Commands.parallel(
                                        manipulator.groundIntakeAlgaeRollersCommand().asProxy(),
                                        arm.moveToPositionCommand(() -> ArmPosition.ALGAE_GROUND_INTAKE).asProxy(),
                                        elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy()),
                                elevator.isStowed)),
                Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-4, 0, 0), DriveMode.ROBOT_RELATIVE), drivetrain)
                        .asProxy().withTimeout(0.15),
                arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE).asProxy())
                .andThen(useRequirement()).withName("RobotCommands.moveToAlgaeIntake");
    }

    // public Command algaeGroundIntakeCommand(Drivetrain drivetrain) {
    // return Commands.sequence(
    // Commands.runOnce(() -> isReadyForNetScoreBool = true),
    // Commands.waitUntil(manipulator.hasAlgae).deadlineFor(
    // Commands.either(
    // Commands.parallel(
    // manipulator.groundIntakeAlgaeRollersCommand().asProxy(),
    // arm.moveToPositionCommand(() -> ArmPosition.ALGAE_GROUND_INTAKE)
    // .asProxy(),
    // Commands.waitUntil(() -> arm.getPosition() < 1.5)
    // .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM)
    // .asProxy())),
    // Commands.parallel(
    // manipulator.groundIntakeAlgaeRollersCommand().asProxy(),
    // arm.moveToPositionCommand(() -> ArmPosition.ALGAE_GROUND_INTAKE)
    // .asProxy(),
    // elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM)
    // .asProxy()),
    // elevator.isStowed)),
    // Commands.run(() -> drivetrain.drive(new ChassisSpeeds(-4, 0, 0),
    // DriveMode.ROBOT_RELATIVE), drivetrain)
    // .asProxy().withTimeout(0.15),
    // arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE).asProxy())
    // .andThen(useRequirement()).withName("RobotCommands.moveToAlgaeIntake");
    // }

    public Command moveToAlgaeReefIntakeCommand(ReefLevel level) {
        ElevatorPosition elevatorPosition;
        switch (level) {
            case L2 -> {
                elevatorPosition = ElevatorPosition.ALGAE_INTAKE_L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.ALGAE_INTAKE_L3;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ReefLevel");
            }
        }

        return Commands.either(
                Commands.parallel(
                        arm.moveToPositionCommand(() -> ArmPosition.ALGAE_REEF_INTAKE).asProxy(),
                        Commands.waitUntil(() -> arm.getPosition() < 2)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())),
                Commands.parallel(
                        arm.moveToPositionCommand(() -> ArmPosition.ALGAE_REEF_INTAKE).asProxy(),
                        elevator.moveToPositionCommand(() -> elevatorPosition).asProxy()),
                elevator.isStowed).andThen(useRequirement())
                .withName("RobotCommands.moveToAlgaeReefIntake");
    }

    public Command algaeReefIntakeCommand(ReefLevel level) {
        return Commands.sequence(
                Commands.runOnce(() -> isInAlgaeReefPickupPositionBool = true),
                Commands.runOnce(() -> isReadyForNetScoreBool = true),
                Commands.deadline(
                        Commands.waitUntil(manipulator.hasAlgae),
                        moveToAlgaeReefIntakeCommand(level)),
                Commands.waitSeconds(0.3),
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.L1).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE).asProxy()))
                .andThen(useRequirement())
                .finallyDo(() -> isInAlgaeReefPickupPositionBool = false)
                .withName("RobotCommands.algaeReefIntake");
    }

    public Command algaeReefIntakeAutoCommand(ReefLevel level) {
        return Commands.sequence(
                Commands.runOnce(() -> isInAlgaeReefPickupPositionBool = true),
                Commands.runOnce(() -> isReadyForNetScoreBool = true),
                Commands.deadline(
                        Commands.waitUntil(manipulator.hasAlgae),
                        moveToAlgaeReefIntakeCommand(level)))
                .andThen(useRequirement())
                .finallyDo(() -> isInAlgaeReefPickupPositionBool = false)
                .withName("RobotCommands.algaeReefIntake");
    }

    // Barge Score
    public Command moveToNetScoreCommand() {
        return Commands.either(
                Commands.parallel(
                        arm.moveToPositionCommand(() -> ArmPosition.BARGE_SCORE).asProxy(),
                        Commands.waitUntil(() -> arm.getPosition() < 2)
                                .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.BARGE_SCORE).asProxy())),
                Commands.parallel(
                        arm.moveToPositionCommand(() -> ArmPosition.BARGE_SCORE).asProxy(),
                        elevator.moveToPositionCommand(() -> ElevatorPosition.BARGE_SCORE).asProxy()),
                elevator.isStowed).andThen(useRequirement()).withName("RobotCommands.moveToBargeScore");
    }

    public Command moveToAutoNetScoreCommand(Drivetrain drivetrain) {
        return Commands.sequence(
                Commands.runOnce(() -> isReadyForNetScoreBool = true),
                Commands.waitUntil(() -> drivetrain.isCloseToNetScorePosition()),
                Commands.either(
                        Commands.parallel(
                                arm.moveToPositionCommand(() -> ArmPosition.AUTO_BARGE_SCORE_START).asProxy(),
                                Commands.waitUntil(() -> arm.getPosition() < 2)
                                        .andThen(elevator.moveToPositionCommand(() -> ElevatorPosition.AUTO_BARGE_SCORE)
                                                .asProxy())),
                        Commands.parallel(
                                arm.moveToPositionCommand(() -> ArmPosition.AUTO_BARGE_SCORE_START).asProxy(),
                                elevator.moveToPositionCommand(() -> ElevatorPosition.AUTO_BARGE_SCORE).asProxy()),
                        elevator.isStowed));
    }

    public Command finishAutoNetScoreCommand() {
        return Commands.parallel(
                arm.moveToPositionCommand(() -> ArmPosition.AUTO_BARGE_SCORE).asProxy(),
                Commands.waitUntil(() -> arm.getPosition() > ArmPosition.AUTO_BARGE_SCORE_RELEASE.value)
                        .andThen(manipulator.reverseRollersCommand().asProxy().withTimeout(1)))
                .withName("finishAutoNetScore");
    }

    public Command finishAutoNetScoreAutoCommand() {
        return Commands.parallel(
                arm.moveToPositionCommand(() -> ArmPosition.AUTO_BARGE_SCORE).asProxy(),
                Commands.waitUntil(() -> arm.getPosition() > ArmPosition.AUTO_BARGE_SCORE_RELEASE.value)
                        .andThen(manipulator.reverseRollersCommand().asProxy().withTimeout(0.4)))
                .withName("finishAutoNetScore");
    }

    // Climb
    public Command prepareClimbCommand(Drivetrain drivetrain) {
        return Commands.parallel(
                elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                Commands.waitUntil(() -> elevator.getPosition() < 0.2)
                        .andThen(arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE).asProxy()),
                drivetrain.controlledRotateCommand(() -> Units.degreesToRadians(90)))
                .andThen(useRequirement()).withName("RobotCommands.prepareClimbCommand");
    }

    public Command afterClimbCommand() {
        return Commands.parallel(
                climber.climbCommand().asProxy(),
                manipulator.stopRollersCommand().asProxy(),
                arm.moveToPositionCommand(() -> ArmPosition.END_CLIMB).asProxy())
                .andThen(useRequirement()).withName("RobotCommands.afterClimbCommand");
    }

    public Command prepareCoralScoreCommand(ReefLevel level) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ReefLevel");
            }
        }

        return setReefSetpointCommand(level).andThen(
                Commands.either(
                        Commands.select(Map.of(
                                ReefLevel.L1,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L1).asProxy(),
                                        elevator.moveToPositionCommand(() -> ElevatorPosition.L1).asProxy()),
                                ReefLevel.L2,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L2).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L2).asProxy())),
                                ReefLevel.L3,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L3).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L3).asProxy())),
                                ReefLevel.L4,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L4).asProxy()))),
                                () -> level),
                        Commands.parallel(
                                arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                elevator.moveToPositionCommand(() -> elevatorPosition).asProxy()),
                        elevator.isStowed))
                .andThen(useRequirement()).withName("RobotCommands.prepareCoralScore");
    }

    public Command prepareCoralScoreFastL4Command(ReefLevel level) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ReefLevel");
            }
        }

        return setReefSetpointCommand(level).andThen(
                Commands.either(
                        Commands.select(Map.of(
                                ReefLevel.L1,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L1).asProxy(),
                                        elevator.moveToPositionCommand(() -> ElevatorPosition.L1).asProxy()),
                                ReefLevel.L2,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L2).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L2).asProxy())),
                                ReefLevel.L3,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L3).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L3).asProxy())),
                                ReefLevel.L4,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L4).asProxy()))

                        ),
                                () -> level),
                        Commands.parallel(
                                arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                elevator.moveToPositionCommand(() -> elevatorPosition).asProxy()),
                        elevator.isStowed))
                .andThen(useRequirement()).withName("RobotCommands.prepareCoralScore");
    }

    public Command prepareCoralScoreHalfwayL4Command(ReefLevel level) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ReefLevel");
            }
        }

        return setReefSetpointCommand(level).andThen(
                Commands.either(
                        Commands.select(Map.of(
                                ReefLevel.L1,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L1).asProxy(),
                                        elevator.moveToPositionCommand(() -> ElevatorPosition.L1).asProxy()),
                                ReefLevel.L2,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L2).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L2).asProxy())),
                                ReefLevel.L3,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> ArmPosition.L3).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L3).asProxy())),
                                ReefLevel.L4,
                                Commands.parallel(
                                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                        Commands.waitSeconds(0.07).andThen(
                                                elevator.moveToPositionCommand(() -> ElevatorPosition.L3).asProxy()))),
                                () -> level),
                        Commands.parallel(
                                arm.moveToPositionCommand(() -> armPosition).asProxy(),
                                elevator.moveToPositionCommand(() -> elevatorPosition).asProxy()),
                        elevator.isStowed))
                .andThen(useRequirement()).withName("RobotCommands.prepareCoralScore");
    }

    public Command scoreProcessorCommand() {
        return Commands.parallel(arm.moveToPositionCommand(() -> ArmPosition.PROCESSOR),
                elevator.moveToPositionCommand(() -> ElevatorPosition.L1)).andThen(useRequirement());
    }

    public Command scoreCoralCommand() {
        Map<ReefLevel, Command> commandMap = Map.of(
                ReefLevel.L1, manipulator.scoreL1Command().asProxy(),
                ReefLevel.L2, manipulator.scoreL23Command().asProxy(),
                ReefLevel.L3, manipulator.scoreL23Command().asProxy(),
                ReefLevel.L4, manipulator.reverseRollersCommand().asProxy());

        return Commands.select(
                commandMap,
                () -> reefSetpoint.orElse(ReefLevel.L4))
                .withTimeout(0.25)
                .andThen(useRequirement()).withName("RobotCommands.scoreCoral");
    }

    public Command scoreCoralContinuouslyCommand() {
        Map<ReefLevel, Command> commandMap = Map.of(
                ReefLevel.L1, manipulator.scoreL1Command().asProxy(),
                ReefLevel.L2, manipulator.scoreL23Command().asProxy(),
                ReefLevel.L3, manipulator.scoreL23Command().asProxy(),
                ReefLevel.L4, manipulator.reverseRollersCommand().asProxy());

        return Commands.select(
                commandMap,
                () -> reefSetpoint.orElse(ReefLevel.L4))
                .andThen(useRequirement()).withName("RobotCommands.scoreCoral");
    }
}
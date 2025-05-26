package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.FieldConstants.Reef.ReefLevel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;

public class Controls {

    public static void configureControls(int port, Drivetrain drivetrain, Superstructure superstructure) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY(),
                        () -> -joystick.getX(),
                        () -> -joystick.getTwist()));

        new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05))
                .whileTrue(drivetrain.disableControlledRotateCommand());

        // Gyro reset field position
        joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

        // Default stow command
        joystick.pinkieButton().onTrue(superstructure.stowCommand());

        // Default score command
        joystick.redButton().whileTrue(superstructure.manipulator.reverseRollersCommand().asProxy())
                .onFalse(superstructure.stowAfterScoreCommand());

        // Align to Coral Station
        joystick.bottomHatLeft()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Units.degreesToRadians(-54)));
        joystick.bottomHatRight()
                .whileTrue(drivetrain.controlledRotateCommand(() -> Units.degreesToRadians(54)));

        // Climb
        joystick.flipTriggerIn().onTrue(superstructure.prepareClimbCommand(drivetrain));
        joystick.flipTriggerIn().and(joystick.blackThumbButton()).onTrue(superstructure.afterClimbCommand());
        joystick.topRightHatDown().whileTrue(superstructure.climber.declimbCommand());

        // Coral Score
        joystick.centerTopHatUp().onTrue(superstructure.prepareCoralScoreCommand(ReefLevel.L4));
        joystick.centerTopHatRight().onTrue(superstructure.prepareCoralScoreCommand(ReefLevel.L3));
        joystick.centerTopHatLeft().onTrue(superstructure.prepareCoralScoreCommand(ReefLevel.L2));
        joystick.centerTopHatDown().onTrue(Commands.either(superstructure.prepareCoralScoreCommand(ReefLevel.L1),
                superstructure.scoreProcessorCommand(), superstructure.isReadyForNetScore.negate()));

        // Reef algae
        joystick.centerBottomHatLeft()
                .onTrue(superstructure.algaeReefIntakeCommand(ReefLevel.L2));
        joystick.centerBottomHatRight()
                .onTrue(superstructure.algaeReefIntakeCommand(ReefLevel.L3));
        joystick.dialSoftPress().onTrue(superstructure.moveToNetScoreCommand());

        // Ground algae
        joystick.centerBottomHatDown()
                .onTrue(superstructure.algaeGroundIntakeCommand(drivetrain));
        joystick.centerBottomHatDown()
                .whileTrue(drivetrain.lockToClosestGroundAlgaeCommand().until(superstructure.manipulator.hasAlgae)
                        .andThen());

        joystick.topRightHatUp().whileTrue(superstructure.arm.moveToPositionCommand(() -> ArmPosition.ELEVATOR_SAFE)
                .alongWith(drivetrain.spinFastCommand()));

        var coralScoringAutoDriveCommands = Commands.either(
                Commands.none(),
                Commands.either(
                        drivetrain.lockToClosestBranchLineCommand(
                                () -> -joystick.getY(), () -> -joystick.getX()),
                        drivetrain.lockToClosestBranchAdaptiveCommand(
                                () -> -joystick.getY(),
                                () -> -joystick.getX(),
                                () -> -joystick.getTwist(),
                                () -> superstructure.getReefSetpoint().orElse(ReefLevel.L4)),
                        GlobalStates.LOCK_TO_LINE_OVERRIDE::enabled),
                GlobalStates.MANUAL_DRIVING_OVERRIDE::enabled);

        joystick.triggerSoftPress().and(joystick.flipTriggerIn().negate()).whileTrue(
                Commands.either(
                        Commands.either(
                                coralScoringAutoDriveCommands,
                                drivetrain.lockToClosestNetScorePositionCommand(
                                        () -> -joystick.getY(),
                                        () -> -joystick.getX(),
                                        () -> -joystick.getTwist())
                                        .alongWith(superstructure.moveToAutoNetScoreCommand(drivetrain)),
                                superstructure.isReadyForNetScore.negate()),
                        Commands.deadline(
                                Commands.waitUntil(superstructure.manipulator.hasAlgae)
                                        .andThen(Commands.waitSeconds(0.8)),
                                drivetrain.lockToClosestReefAlgaeAdaptiveCommand(() -> -joystick.getY(),
                                        () -> -joystick.getX(), () -> -joystick.getTwist(),
                                        superstructure.manipulator.hasAlgae)),
                        superstructure.isInAlgaeReefPickupPosition.negate()));

        joystick.triggerHardPress().and(joystick.flipTriggerIn().negate()).onTrue(
                Commands.either(
                        Commands.waitUntil(superstructure.isReadyToScore)
                                .andThen(Commands.waitSeconds(0.1))
                                .andThen(superstructure.finishAutoNetScoreCommand()),
                        superstructure.scoreCoralContinuouslyCommand().withTimeout(1),
                        superstructure.isReadyForNetScore));

        joystick.triggerHardPress().and(joystick.flipTriggerIn().negate())
                .onFalse(superstructure.stowAfterScoreCommand());

        // TODO: reenable once coral trigger is better
        // elevator.isStowed.and(manipulator.hasGamePiece).onTrue(arm.moveToPositionCommand(()
        // -> ArmPosition.SAFE_CORAL));

    }

    public static void configureTestingControls(int port, Drivetrain drivetrain, Superstructure superstructure) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.povLeft().whileTrue(
                superstructure.arm.moveToArbitraryPositionCommand(() -> superstructure.arm.getPosition() - 0.1));
        controller.povRight().whileTrue(
                superstructure.arm.moveToArbitraryPositionCommand(() -> superstructure.arm.getPosition() + 0.1));
        controller.povDown().whileTrue(
                superstructure.elevator
                        .moveToArbitraryPositionCommand(() -> superstructure.elevator.getPosition() - 0.1));
        controller.povUp().whileTrue(
                superstructure.elevator
                        .moveToArbitraryPositionCommand(() -> superstructure.elevator.getPosition() + 0.1));

        controller.rightTrigger().whileTrue(superstructure.manipulator.runRollersCommand());
        controller.leftTrigger().whileTrue(superstructure.manipulator.reverseRollersCommand());

        controller.povDown().whileTrue(
                superstructure.arm.moveToArbitraryPositionCommand(() -> superstructure.arm.getPosition() - 0.1));
        controller.povUp().whileTrue(
                superstructure.arm.moveToArbitraryPositionCommand(() -> superstructure.arm.getPosition() + 0.1));

        controller.y().onTrue(Commands.parallel(
                GlobalStates.LOCK_TO_LINE_OVERRIDE.disableCommand(),
                GlobalStates.MANUAL_DRIVING_OVERRIDE.disableCommand()));
        controller.b().onTrue(Commands.parallel(
                GlobalStates.LOCK_TO_LINE_OVERRIDE.enableCommand(),
                GlobalStates.MANUAL_DRIVING_OVERRIDE.disableCommand()));
        controller.b().onTrue(Commands.parallel(
                GlobalStates.LOCK_TO_LINE_OVERRIDE.disableCommand(),
                GlobalStates.MANUAL_DRIVING_OVERRIDE.enableCommand()));

        // controller.x().whileTrue(superstructure.elevator.sysIdQuasistaticCommand(Direction.kForward));
        // controller.y().whileTrue(superstructure.elevator.sysIdQuasistaticCommand(Direction.kReverse));
        // controller.a().whileTrue(superstructure.elevator.sysIdDynamicCommand(Direction.kForward));
        // controller.b().whileTrue(superstructure.elevator.sysIdDynamicCommand(Direction.kReverse));

        // superstructure.elevator
        // .setDefaultCommand(superstructure.elevator.setOverridenSpeedCommand(() ->
        // -0.5 *
        // controller.getLeftY()));
        // superstructure.arm
        // .setDefaultCommand(superstructure.arm.setOverridenSpeedCommand(() -> -0.5 *
        // controller.getRightY()));

        controller.a().onTrue(superstructure.stowAfterScoreCommand());
        controller.y().onTrue(superstructure.prepareCoralScoreCommand(ReefLevel.L4));
        controller.b().onTrue(superstructure.prepareCoralScoreCommand(ReefLevel.L2));

        controller.rightBumper().onTrue(
                Commands.parallel(
                        drivetrain.manualInitializeCommand(),
                        superstructure.elevator.manualInitializeCommand(),
                        superstructure.arm.manualInitializeCommand(),
                        superstructure.climber.manualInitializeCommand())
                        .ignoringDisable(true));

        controller.leftBumper().and(DriverStation::isDisabled).onTrue(Commands.parallel(
                drivetrain.resetGyroCommand(),
                superstructure.elevator.resetPositionCommand(),
                superstructure.arm.resetPositionCommand(),
                superstructure.climber.resetPositionCommand()).ignoringDisable(true));

    }

}
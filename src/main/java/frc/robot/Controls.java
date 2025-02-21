package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Level;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

/**
 * Controls configurations for robot.
 */
public class Controls {
    public static final class Constants {
        enum ControllerType {
            XboxController,
            FlightStick
        }

        public static final boolean DEBUG_MODE = false;

        public static final ControllerType CONTROLLER_TYPE = ControllerType.FlightStick;

        public static final double PERIOD = 0.020;

        public static final class Teleop {
            /**
             * A value inputted into the rate limiter (the joystick input) can move from 0
             * to 1 in 1/RATE_LIMIT seconds.
             * 
             * A rate limit of 3, for example, means that 0->1 in 1/3 sec.
             * Larger numbers mean less of a rate limit.
             */
            public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0; // TODO
            public static final double JOYSTICK_INPUT_DEADBAND = 0.05; // TODO
            public static final double JOYSTICK_CURVE_EXP = 2; // TODO
            public static final double JOYSTICK_ROT_CURVE_EXP = 1; // TODO
        }
    }

    /**
     * Configure driver controls on a Virpil Controls Alpha-R joystick.
     * 
     * @param port       port that joystick is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureDriverControls(int port, Drivetrain drivetrain, Elevator elevator,
            Pivot pivot, Manipulator manipulator, Intake intake, Climber climber, LEDs leds) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(() -> -joystick.getY(), () -> -joystick.getX(),
                        () -> -joystick.getTwist()));

        joystick.flipTriggerIn().onTrue(RobotCommands.setModeCommand(Mode.ALGAE));
        joystick.flipTriggerOut().onTrue(RobotCommands.setModeCommand(Mode.CORAL));

        joystick.bottomHatDown()
                .onTrue(RobotCommands.setScoringTargetLevelCommand(RobotStates::getMode, Level.L1, Level.PROCESSOR));
        joystick.bottomHatLeft()
                .onTrue(RobotCommands.setScoringTargetLevelCommand(RobotStates::getMode, Level.L2, Level.PROCESSOR));
        joystick.bottomHatUp()
                .onTrue(RobotCommands.setScoringTargetLevelCommand(RobotStates::getMode, Level.L3, Level.NET));
        joystick.bottomHatRight()
                .onTrue(RobotCommands.setScoringTargetLevelCommand(RobotStates::getMode, Level.L4, Level.NET));

        joystick.triggerSoftPress()
                .onTrue(RobotCommands.moveToScoreCommand(RobotStates::getTargetLevel,
                        elevator, pivot));
        joystick.triggerHardPress().onTrue(manipulator.reverseRollersCommand())
                .toggleOnFalse(RobotCommands.rehomeMechanismsCommand(elevator, pivot, manipulator));

        joystick.redButton().whileTrue(climber.setCurrentCommand());

        joystick.blackThumbButton().onTrue(manipulator.runRollersCommand().until(manipulator::hasScoringElement));
    }

    /**
     * Configure operator controls on Xbox controller for manual overrides in
     * caseautomatic features stop working.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureOperatorControls(int port, Drivetrain drivetrain, Elevator elevator,
            Pivot pivot, Manipulator manipulator, Intake intake, Climber climber, LEDs leds) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.y().whileTrue(elevator.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.1));
        controller.b().whileTrue(pivot.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25));
        controller.a().whileTrue(intake.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25));
        controller.x().whileTrue(climber.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25));

        controller.rightBumper().whileTrue(manipulator.runRollersCommand());
        controller.leftBumper().whileTrue(manipulator.reverseRollersCommand());

        controller.rightTrigger().whileTrue(intake.setOverridenSpeedCommand(() -> 0.5));
        controller.leftTrigger().whileTrue(intake.setOverridenSpeedCommand(() -> -0.5));
    }

    /**
     * Configure override controls on Xbox controller for overriding mechanism
     * safeties.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureOverrideControls(int port, Drivetrain drivetrain, Elevator elevator,
            Pivot pivot, Manipulator manipulator, Intake intake, Climber climber, LEDs leds) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().whileTrue(intake.runRollersCommand());
        controller.y().whileTrue(intake.reverseRollersCommand());
    }

    /**
     * Configure controls on Xbox controller for running SysId routines.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureSysIdControls(int port, Drivetrain drivetrain, Elevator elevator,
            Pivot pivot, Manipulator manipulator, Intake intake, Climber climber, LEDs leds) {
        CommandXboxController controller = new CommandXboxController(port);

        controller.x().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kForward));
        controller.y().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveQuasistatic(Direction.kReverse));
        controller.a().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kForward));
        controller.b().and(controller.povUp()).whileTrue(drivetrain.sysIdDriveDynamic(Direction.kReverse));

        controller.x().and(controller.povDown()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povDown()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povDown()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povDown()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));

        controller.x().and(controller.povLeft()).whileTrue(pivot.sysIdQuasistatic(Direction.kForward));
        controller.y().and(controller.povLeft()).whileTrue(pivot.sysIdQuasistatic(Direction.kReverse));
        controller.a().and(controller.povLeft()).whileTrue(pivot.sysIdDynamic(Direction.kForward));
        controller.b().and(controller.povLeft()).whileTrue(pivot.sysIdDynamic(Direction.kReverse));
    }
}

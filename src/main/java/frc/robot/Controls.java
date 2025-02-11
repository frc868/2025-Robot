package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Controls configurations for each member of drive team.
 * Each member has a static method which configures their controls.
 */
public class Controls {

        private static boolean alignLeft = false; // default to aligning to the rightmost coral
        private static boolean inCoralMode = true; // default to coral mode
        private static int level = 1; // default to level 1

        public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds) {
                CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);
                new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05))
                                .whileTrue(drivetrain.disableControlledRotateCommand());

                drivetrain.setDefaultCommand(
                                drivetrain.teleopDriveCommand(
                                                () -> -joystick.getY() * Constants.DEMO_SPEED.get(),
                                                () -> -joystick.getX() * Constants.DEMO_SPEED.get(),
                                                () -> -joystick.getTwist() * Constants.DEMO_SPEED.get()));

                joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

                // Align to left or right coral
                joystick.centerTopHatLeft().onTrue(
                                changeAlignLeft(true));
                joystick.centerTopHatRight().onTrue(
                                changeAlignLeft(false));

                // Change mode
                joystick.flipTriggerIn().onTrue(
                                changeMode(false));
                joystick.flipTriggerOut().onTrue(
                                changeMode(true));

                // if in coral mode, go to correct position
                joystick.bottomHatUp().and(() -> inCoralMode).onTrue(Commands.parallel(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L4),
                                changeLevel(4)));
                joystick.bottomHatRight().and(() -> inCoralMode).onTrue(Commands.parallel(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L3), changeLevel(3)));
                joystick.bottomHatDown().and(() -> inCoralMode).onTrue(Commands.parallel(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L2), changeLevel(2)));
                joystick.bottomHatLeft().and(() -> inCoralMode).onTrue(Commands.parallel(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L1), changeLevel(1)));
                // if in coral mode, go to correct position
                joystick.bottomHatUp().and(() -> !inCoralMode).onTrue(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.NET));
                joystick.bottomHatDown().and(() -> !inCoralMode).onTrue(
                                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.PROCESSOR));

                // joystick.centerBottomHatUp().onTrue(intake.intakeCommand());
        }

        /**
         * changes the alignment to left or right coral
         * 
         * @param align true is left, false is right
         * @return
         */
        public static Command changeAlignLeft(boolean align) {
                return Commands.runOnce(() -> alignLeft = align);
        }

        public static Command changeLevel(int level) {
                return Commands.runOnce(() -> Controls.level = level);
        }

        /**
         * changes the current operating mode
         * 
         * @param mode true is coral, false is algae
         */
        public static Command changeMode(boolean mode) {
                return Commands.runOnce(() -> inCoralMode = mode);
        }

        public static void configureOperatorControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds) {

        }

        public static void configureOverridesControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds) {

        }

        public static void configureTestingControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds) {

        }

}

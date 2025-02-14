package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotCommands;

/**
 * Controls configurations for each member of drive team.
 * Each member has a static method which configures their controls.
 */
public class Controls {

        private static boolean alignLeft = false; // default to aligning to the rightmost coral
        private static boolean inCoralMode = true; // default to coral mode
        private static int level = 1; // default to level 1
        private static boolean climbReady = false;

        public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds, Vision vision) {
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
                joystick.bottomHatUp().and(() -> inCoralMode)
                                .onTrue(RobotCommands.moveToCoralScoringPositionCommand(pivot, elevator, 4)
                                                .andThen(changeLevel(4)));
                joystick.bottomHatRight().and(() -> inCoralMode)
                                .onTrue(RobotCommands.moveToCoralScoringPositionCommand(pivot, elevator, 3)
                                                .andThen(changeLevel(3)));
                joystick.bottomHatDown().and(() -> inCoralMode)
                                .onTrue(RobotCommands.moveToCoralScoringPositionCommand(pivot, elevator, 2)
                                                .andThen(changeLevel(2)));
                joystick.bottomHatLeft().and(() -> inCoralMode)
                                .onTrue(RobotCommands.moveToCoralScoringPositionCommand(pivot, elevator, 1)
                                                .andThen(changeLevel(1)));

                // if in coral mode, go to correct position
                joystick.bottomHatUp().and(() -> !inCoralMode)
                                .onTrue(RobotCommands.moveToAlgaeScoringPositionCommand(pivot, elevator, 5)
                                                .andThen(changeLevel(5)));
                joystick.bottomHatDown().and(() -> !inCoralMode)
                                .onTrue(RobotCommands.moveToAlgaeScoringPositionCommand(pivot, elevator, 2)
                                                .andThen(changeLevel(2)));

                // joystick.centerBottomHatUp().onTrue(intake.intakeCommand());

                // when the red button is pressed, prepare for the climb, then wait for a second
                // press to climb
                joystick.redButton().and(() -> !climbReady)
                                .onTrue(RobotCommands.prepareClimbCommand(intake, elevator, pivot, climber));
                joystick.redButton().and(() -> climbReady)
                                .onTrue(RobotCommands.climbCommand(climber));

                // intake algae from reef
                joystick.blackThumbButton()
                                .whileTrue(RobotCommands.intakeAlgaeReefCommand(pivot, manipulator, elevator, 3 / 4)); // TODO
                                                                                                                       // some
                                                                                                                       // way
                                                                                                                       // to
                                                                                                                       // change
                                                                                                                       // reef
                                                                                                                       // levels
                // intake/eject algae from the ground
                joystick.centerBottomHatUp().whileTrue(
                                RobotCommands.intakeAlgaeGroundCommand(pivot, manipulator, elevator, intake));
                joystick.centerBottomHatDown().whileTrue(
                                RobotCommands.ejectAlgaeGroundCommand(pivot, manipulator, elevator, intake));

                // trigger presses
                // lock to target
                joystick.triggerSoftPress().and(() -> inCoralMode)
                                .whileTrue(RobotCommands.lockOnCommand(drivetrain, vision, 1, alignLeft));
                joystick.triggerSoftPress().and(() -> !inCoralMode).and(() -> level == 5)
                                .whileTrue(RobotCommands.lockOnCommand(drivetrain, vision, 2, alignLeft));
                joystick.triggerSoftPress().and(() -> !inCoralMode).and(() -> level == 2)
                                .whileTrue(RobotCommands.lockOnCommand(drivetrain, vision, 3, alignLeft));

                // score coral/algae
                joystick.triggerHardPress().and(() -> inCoralMode)
                                .onTrue(RobotCommands.scoreCoralCommand(pivot, elevator, manipulator));
                joystick.triggerHardPress().and(() -> !inCoralMode)
                                .onTrue(RobotCommands.scoreAlgaeCommand(pivot, elevator, manipulator));

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

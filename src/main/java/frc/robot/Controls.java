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

/**
 * Controls configurations for each member of drive team.
 * Each member has a static method which configures their controls.
 */
public class Controls {

        public static void configureDriverControls(int port, Drivetrain drivetrain, Intake intake, Elevator elevator,
                        Pivot pivot, Manipulator manipulator, Climber climber, LEDs leds) {
                CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);
                new Trigger(() -> (Math.abs(joystick.getTwist()) > 0.05))
                                .whileTrue(drivetrain.disableControlledRotateCommand());

                joystick.stickButton().onTrue(drivetrain.resetGyroCommand());

                joystick.centerBottomHatUp()
                                .whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L4));
                joystick.centerBottomHatDown()
                                .whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L2));
                joystick.centerBottomHatLeft()
                                .whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L1));
                joystick.centerBottomHatRight()
                                .whileTrue(elevator.moveToPositionCommand(() -> Elevator.Constants.Position.L3));

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

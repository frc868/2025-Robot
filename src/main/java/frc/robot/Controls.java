package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

/**
 * Controls configurations for each member of drive team.
 * Each member has a static method which configures their controls.
 */
public class Controls {
    public static final class Constants {
        public static final TunableDouble DEMO_SPEED = new TunableDouble("subsystems/drivetrain/DEMO_SPEED", 0.5);

    }

    /**
     * Configures driver controls on a Virpil Controls Alpha-R joystick.
     * 
     * @param port        the port that the joystick is connected to
     * @param drivetrain  the drivetrain
     * @param intake      the intake
     * @param shooter     the shooter
     * @param shooterTilt the shooter tilt
     * @param climber     the climber
     * @param leds        the LEDs
     */
    public static void configureDriverControls(int port, Drivetrain drivetrain) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * Constants.DEMO_SPEED.get(),
                        () -> -joystick.getX() * Constants.DEMO_SPEED.get(),
                        () -> -joystick.getTwist() * Constants.DEMO_SPEED.get()));
    }

}

package frc.robot;

import com.techhounds.houndutil.houndlib.oi.CommandVirpilJoystick;
import com.techhounds.houndutil.houndlog.loggers.TunableDouble;

import frc.robot.subsystems.Drivetrain;

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

        public static final TunableDouble DEMO_SPEED = new TunableDouble("DEMO_SPEED", 1.0);
    }

    /**
     * Configure driver controls on a Virpil Controls Alpha-R joystick.
     * 
     * @param port       port that joystick is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureDriverControls(int port, Drivetrain drivetrain) {
        CommandVirpilJoystick joystick = new CommandVirpilJoystick(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -joystick.getY() * Constants.DEMO_SPEED.get(),
                        () -> -joystick.getX() * Constants.DEMO_SPEED.get(),
                        () -> -joystick.getTwist() * Constants.DEMO_SPEED.get()));
    }

    /**
     * Configure operator controls on Xbox controller for manual overrides in case
     * automatic features stop working.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureOperatorControls(int port, Drivetrain drivetrain) {
    }

    /**
     * Configure override controls on Xbox controller for overriding mechanism
     * safeties.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureOverrideControls(int port, Drivetrain drivetrain) {
    }

    /**
     * Configure controls on Xbox controller for running SysId routines.
     * 
     * @param port       port that controller is connected to
     * @param drivetrain drivetrain subsystem object
     */
    public static void configureSysIdControls(int port, Drivetrain drivetrain) {
    }
}

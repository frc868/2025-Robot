package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.techhounds.houndutil.houndlib.AprilTagPhotonCamera.PhotonCameraConstants;
import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
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

        public static final class Field {
                public static final double FIELD_LENGTH = Units.inchesToMeters(690.88);
                public static final double FIELD_WIDTH = Units.inchesToMeters(317.15);

                public static final Pose3d PROCESSOR_OPENING = new Pose3d(Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0), new Rotation3d());

                public static final Pose3d LEFT_CORAL_STATION = new Pose3d(Units.inchesToMeters(0),
                                Units.inchesToMeters(0),
                                Units.inchesToMeters(0), new Rotation3d());

                public static final Pose3d RIGHT_CORAL_STATION = new Pose3d(Units.inchesToMeters(0),
                                Units.inchesToMeters(0), Units.inchesToMeters(0),
                                new Rotation3d());

                public static final Pose2d REEF_CENTER = new Pose2d(Units.inchesToMeters(144.0 + 65.49 / 2.0),
                                FIELD_WIDTH / 2.0, new Rotation2d());

                public static final double REEF_L1_HEIGHT = 0.46;
                public static final double REEF_L2_HEIGHT = 0.81;
                public static final double REEF_L3_HEIGHT = 1.21;
                public static final double REEF_L4_HEIGHT = 1.83;

                public static final double REEF_LEVEL_HEIGHTS[] = { REEF_L1_HEIGHT, REEF_L2_HEIGHT, REEF_L3_HEIGHT,
                                REEF_L4_HEIGHT };

                public static final int REEF_SIDES = 6;
                public static final int REEF_PLACES_PER_SIDE = 2;
                public static final double REEF_RADIUS = Units.inchesToMeters(65.49) / 2.0; // Inscribed circle radius
                                                                                            // of the reef
                public static final double REEF_RADIUS_OFFSET = Units.inchesToMeters(12.94) / 2.0; // Distance each reef
                                                                                                   // place is from the
                                                                                                   // radius above

                public static final double REEF_L1_INSET = 0.1;
                public static final double REEF_L2_INSET = 0.041;
                public static final double REEF_L3_INSET = 0.041;
                public static final double REEF_L4_INSET = 0.027;

                public static final double REEF_LEVEL_RADII[] = { REEF_RADIUS - REEF_L1_INSET,
                                REEF_RADIUS - REEF_L2_INSET, REEF_RADIUS - REEF_L3_INSET, REEF_RADIUS - REEF_L4_INSET };

                public static final double REEF_L1_ANGLE = Units.degreesToRadians(10.0);
                public static final double REEF_L2_ANGLE = Units.degreesToRadians(-35.0);
                public static final double REEF_L3_ANGLE = Units.degreesToRadians(-35.0);
                public static final double REEF_L4_ANGLE = Units.degreesToRadians(-90.0);

                public static final double REEF_LEVEL_ANGLES[] = { REEF_L1_ANGLE, REEF_L2_ANGLE, REEF_L3_ANGLE,
                                REEF_L4_ANGLE };
        }

        public static final class HoundBrian {
                public static final int BUTTON_1 = 3;
                public static final int BUTTON_2 = 4;
                public static final int BUTTON_3 = 5;
                public static final int BUTTON_4 = 6;
                public static final int BUTTON_5 = 7;
                public static final int BUTTON_6 = 8;
                public static final int BUTTON_7 = 9;
        }
}

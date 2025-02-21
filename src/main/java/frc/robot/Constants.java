package frc.robot;

import java.util.List;

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
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.LEDs.LEDState;

import com.techhounds.houndutil.houndlib.swerve.CoaxialSwerveModule.SwerveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class Constants {
    public static final class Field {
        public static final int[] sideToTagIDBlue = { 18, 17, 22, 21, 20, 19 };
        public static final int[] sideToTageIDRed = { 7, 8, 9, 10, 11, 6 };

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

        public static final double REEF_INITIAL_POINT_RADIUS = REEF_RADIUS + 1.5;

        public static final double REEF_SCORING_POS_OFFSET_FROM_FROM_APRILTAG = 0.5;

        public static final double REEF_LEVEL_RADII[] = { REEF_RADIUS - REEF_L1_INSET,
                REEF_RADIUS - REEF_L2_INSET, REEF_RADIUS - REEF_L3_INSET, REEF_RADIUS - REEF_L4_INSET };

        public static final double REEF_L1_ANGLE = Units.degreesToRadians(10.0);
        public static final double REEF_L2_ANGLE = Units.degreesToRadians(-35.0);
        public static final double REEF_L3_ANGLE = Units.degreesToRadians(-35.0);
        public static final double REEF_L4_ANGLE = Units.degreesToRadians(-90.0);

        public static final double REEF_LEVEL_ANGLES[] = { REEF_L1_ANGLE, REEF_L2_ANGLE, REEF_L3_ANGLE,
                REEF_L4_ANGLE };
    }

    public static enum Level {
        L1(Elevator.Constants.Position.L1, Pivot.Constants.Position.L1),
        L2(Elevator.Constants.Position.L2, Pivot.Constants.Position.L2),
        L3(Elevator.Constants.Position.L3, Pivot.Constants.Position.L3),
        L4(Elevator.Constants.Position.L4_NET, Pivot.Constants.Position.L4),
        GROUND(Elevator.Constants.Position.GROUND_ALGAE, Pivot.Constants.Position.GROUND_ALGAE),
        PROCESSOR(Elevator.Constants.Position.PROCESSOR, Pivot.Constants.Position.PROCESSOR),
        REEF_LOW_ALGAE(Elevator.Constants.Position.REEF_LOW_ALGAE, Pivot.Constants.Position.ALGAE),
        REEF_HIGH_ALGAE(Elevator.Constants.Position.REEF_HIGH_ALGAE, Pivot.Constants.Position.ALGAE),
        NET(Elevator.Constants.Position.L4_NET, Pivot.Constants.Position.NET);

        public final Elevator.Constants.Position elevatorPosition;
        public final Pivot.Constants.Position pivotPosition;

        private Level(Elevator.Constants.Position elevatorPosition,
                Pivot.Constants.Position pivotPosition) {
            this.elevatorPosition = elevatorPosition;
            this.pivotPosition = pivotPosition;
        }
    }

    public static enum Mode {
        CORAL,
        ALGAE;
    }

    public static final class LEDs {
        public static enum LEDSection implements BaseLEDSection {
            ALL(0, 326, true);

            private final int startIdx;
            private final int endIdx;
            private final boolean inverted;

            private LEDSection(int startIdx, int endIdx, boolean inverted) {
                this.startIdx = startIdx;
                this.endIdx = endIdx;
                this.inverted = inverted;
            }

            public int start() {
                return startIdx;
            }

            public int end() {
                return endIdx;
            }

            public boolean inverted() {
                return inverted;
            }

            public int length() {
                return endIdx - startIdx + 1;
            }
        }

        public static final int PORT = 0; // Change depending on PWM
        public static final int LENGTH = 310; // Adjust as needed

        public static final List<LEDState> DEFAULT_STATES = List.of();
    }
}
package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Field.*;
import frc.robot.Constants.*;

public class TargetAlign {

    /**
     * Gets the position of the designated target on the field. Returned rotation is
     * intended to be yawed first by drivetrain and then pitched by arm.
     *
     * @param level    The level of the target.
     * @param position The position of the target.
     * @return The Pose3d of the target.
     */
    public Pose3d getReefPositionPose(int level, int position) {
        double height = Field.REEF_LEVEL_HEIGHTS[level];
        double radius = Field.REEF_LEVEL_RADII[level];
        double verticalAngle = Field.REEF_LEVEL_ANGLES[level]; // 0 is horizontal, positive is angle up

        int side = position / Field.REEF_PLACES_PER_SIDE;
        int place = position % Field.REEF_PLACES_PER_SIDE;

        double horizontalAngle = 2.0 * Math.PI / Field.REEF_SIDES * side; // 0 is facing the -X direction, moving
                                                                          // counterclockwise. Will normalize with
                                                                          // convention (+X is 0) on return
        double offsetDirection = (place == 0) ? -1 : 1;

        double x = -radius * Math.cos(horizontalAngle)
                + offsetDirection * Field.REEF_RADIUS_OFFSET * Math.sin(horizontalAngle);
        double y = -radius * Math.sin(horizontalAngle)
                - offsetDirection * Field.REEF_RADIUS_OFFSET * Math.cos(horizontalAngle);

        var alliance = DriverStation.getAlliance();
        boolean invert = false;

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = true;
        }

        if (invert) {
            Pose2d pos = getRedPosition(new Pose2d(x, y, new Rotation2d(horizontalAngle)));
            x = pos.getX();
            y = pos.getY();
            horizontalAngle = pos.getRotation().getRadians();
        }

        Pose3d pose = new Pose3d(x, y, height, new Rotation3d(0, verticalAngle, horizontalAngle + (Math.PI)));
        return pose;
    }

    /**
     * Converts from blue alliance position to red alliance position. Takes the
     * difference of the center of the field and the blue position, and adds it back
     * to the center position. Also adds 180 degrees to angle.
     *
     * @param bluePosition The Pose2d of the blue alliance position.
     * @return The Pose2d of the red alliance position.
     */
    public Pose2d getRedPosition(Pose2d bluePosition) {
        double x = bluePosition.getX();
        double y = bluePosition.getY();
        double angle = bluePosition.getRotation().getRadians();

        double distanceFromCenterX = Field.FIELD_LENGTH / 2.0 - x;
        double distanceFromCenterY = Field.FIELD_WIDTH / 2.0 - y;

        x = Field.FIELD_LENGTH / 2.0 + distanceFromCenterX;
        y = Field.FIELD_WIDTH / 2.0 + distanceFromCenterY;

        angle = angle + Math.PI;

        return new Pose2d(x, y, new Rotation2d(angle));
    }

}

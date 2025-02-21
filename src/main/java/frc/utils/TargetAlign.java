package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Field.*;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Transform2d;

public class TargetAlign {

    /**
     * Gets the position of the designated target on the field. Returned rotation is
     * intended to be yawed first by drivetrain and then pitched by arm.
     *
     * @param level    The level of the target.
     * @param position The position of the target.
     * @return The Pose3d of the target.
     */
    public static Pose3d getReefPositionPose(int level, int position) {
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
     * Gets the rotation of the designated target on the field.
     * 
     * @return The Rotation2d of the target.
     */
    public static Rotation2d getTargetRotation(int side) {
        double horizontalAngle = 2.0 * Math.PI / Field.REEF_SIDES * side + Math.PI;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            horizontalAngle += Math.PI;
        }
        return new Rotation2d(horizontalAngle);
    }

    /**
     * Gets the offset from the apriltag to the target, fieldrelative.
     * 
     * @return The Transform2d of the target.
     */
    public static Transform2d getTargetTransformFromAprilTag(boolean left, int side) {
        int offsetDirection = left ? 1 : -1;
        double x = offsetDirection * Field.REEF_RADIUS_OFFSET * Math.sin(getTargetRotation(side).getRadians())
                + Field.REEF_SCORING_POS_OFFSET_FROM_FROM_APRILTAG * Math.cos(getTargetRotation(side).getRadians());
        double y = -offsetDirection * Field.REEF_RADIUS_OFFSET * Math.cos(getTargetRotation(side).getRadians())
                + Field.REEF_SCORING_POS_OFFSET_FROM_FROM_APRILTAG * Math.sin(getTargetRotation(side).getRadians());
        return new Transform2d(x, y, new Rotation2d(0));
    }

    /**
     * Gets the closest side to the robot's estimated location.
     * 
     * @param estimatedLocation The estimated location of the robot.
     * @return The closest side to the robot's estimated location.
     */
    public static int getClosestSide(Pose2d estimatedLocation) {
        double minDistance = Double.MAX_VALUE;
        int closestSide = 0;

        for (int i = 0; i < Field.REEF_SIDES; i++) {
            double radius = Field.REEF_LEVEL_RADII[0];

            int side = i;
            double horizontalAngle = 2.0 * Math.PI / Field.REEF_SIDES * side; // 0 is facing the -X direction, moving
                                                                              // counterclockwise. Will normalize with
                                                                              // convention (+X is 0) on return
            double x = -radius * Math.cos(horizontalAngle);
            double y = -radius * Math.sin(horizontalAngle);

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
            Pose2d pose = new Pose2d(x, y, new Rotation2d(horizontalAngle + (Math.PI)));

            double distance = pose.getTranslation().getDistance(estimatedLocation.getTranslation());
            if (distance < minDistance) {
                minDistance = distance;
                closestSide = i;
            }
        }
        return closestSide;
    }

    /**
     * Gets the initial alignment point for the targeted side. This point is used
     * when the cameras cannot see the target Apriltag
     * 
     * @param side side to align to. Follows ABC, etc order.
     * @return Pose2d of the initial point with proper rotation to see the target
     *         apriltag.
     */
    public static Pose2d getInitialPoint(int side) {

        double radius = Field.REEF_INITIAL_POINT_RADIUS;

        double horizontalAngle = 2.0 * Math.PI / Field.REEF_SIDES * side; // 0 is facing the -X direction, moving
                                                                          // counterclockwise. Will normalize with
                                                                          // convention (+X is 0) on return
        double x = -radius * Math.cos(horizontalAngle);
        double y = -radius * Math.sin(horizontalAngle);

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

        Pose2d pose = new Pose2d(x, y, new Rotation2d(horizontalAngle + (Math.PI)));
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
    public static Pose2d getRedPosition(Pose2d bluePosition) {
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

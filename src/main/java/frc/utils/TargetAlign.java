package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class TargetAlign {

    //gets the position of the destinated target on the field. Returned rotation is intended to be yawed first by drivetrain and then pitched by arm. 
    public Pose3d getReefPositionPose(int level, int position) {
        double height = FieldConstants.REEF_LEVEL_HEIGHTS[level];
        double radius = FieldConstants.REEF_LEVEL_RADII[level];
        double verticalAngle = FieldConstants.REEF_LEVEL_ANGLES[level]; //0 is horizontal, positive is angle up

        int side = position / FieldConstants.REEF_PLACES_PER_SIDE;
        int place = position % FieldConstants.REEF_PLACES_PER_SIDE;

        double horizontalAngle = 2.0 * Math.PI / FieldConstants.REEF_SIDES * side; //0 is facing the -X direction, moving counterclockwise. Will normalize with convention (+X is 0) on return
        double offsetDirection = (place == 0) ? -1 : 1;

        double x = -radius * Math.cos(horizontalAngle) + offsetDirection * FieldConstants.REEF_RADIUS_OFFSET * Math.sin(horizontalAngle);
        double y = -radius * Math.sin(horizontalAngle) - offsetDirection * FieldConstants.REEF_RADIUS_OFFSET * Math.cos(horizontalAngle);


        var alliance = DriverStation.getAlliance();
        boolean invert = false;

        if(alliance.isPresent() && alliance.get() == Alliance.Red) {
            invert = true;
        }

        if(invert) {
            Pose2d pos = getRedPosition(new Pose2d(x, y, new Rotation2d(horizontalAngle)));
            x = pos.getX();
            y = pos.getY();
            horizontalAngle = pos.getRotation().getRadians();
        }

        Pose3d pose = new Pose3d(x, y, height, new Rotation3d(0,verticalAngle, horizontalAngle + (Math.PI)));
        return pose;
    }

    //Converts from blue alliance pos to red alliance pos. Takes the difference of the center of the field and the blue position, and adds it back to the center position. Also adds 180 degrees to angle. 
    public Pose2d getRedPosition(Pose2d bluePosition) {
        double x = bluePosition.getX();
        double y = bluePosition.getY();
        double angle = bluePosition.getRotation().getRadians();

        double distanceFromCenterX = FieldConstants.FIELD_LENGTH / 2.0 - x;
        double distanceFromCenterY = FieldConstants.FIELD_WIDTH / 2.0 - y;

        x = FieldConstants.FIELD_LENGTH / 2.0 + distanceFromCenterX;
        y = FieldConstants.FIELD_WIDTH / 2.0 + distanceFromCenterY;

        angle = angle + Math.PI;

        return new Pose2d(x, y, new Rotation2d(angle));
    }
    
}

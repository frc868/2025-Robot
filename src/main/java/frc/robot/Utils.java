package frc.robot;

import com.techhounds.houndutil.houndauto.Reflector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utils {
    public static Pose2d rotateBluePoseIfNecessary(Pose2d original) {
        return shouldFlipValueToRed()
                ? Reflector.rotatePoseAcrossField(original, FieldConstants.fieldLength, FieldConstants.fieldWidth)
                : original;
    }

    public static double getLineDistance(Pose2d pose, Pose2d lineApexPose) {
        double cosTheta = lineApexPose.getRotation().getCos();
        double sinTheta = lineApexPose.getRotation().getSin();

        double A = -sinTheta;
        double B = cosTheta;
        double C = -(A * lineApexPose.getX() + B * lineApexPose.getY());

        // formula for distance between point and line given Ax + By + C = 0
        double distance = (A * pose.getX() + B * pose.getY() + C)
                / Math.sqrt(A * A + B * B);
        return distance;
    }

    public static Pose2d getClosestPoseOnLine(Pose2d pose, Pose2d lineApexPose) {
        double cosTheta = lineApexPose.getRotation().getCos();
        double sinTheta = lineApexPose.getRotation().getSin();

        double A = -sinTheta;
        double B = cosTheta;
        double C = -(A * lineApexPose.getX() + B * lineApexPose.getY());

        // Project the point onto the line using the formula for closest point
        // projection
        double xClosest = (B * (B * pose.getX() - A * pose.getY()) - A * C) / (A * A + B * B);
        double yClosest = (A * (-B * pose.getX() + A * pose.getY()) - B * C) / (A * A + B * B);

        return new Pose2d(xClosest, yClosest, lineApexPose.getRotation());
    }

    public static boolean shouldFlipValueToRed() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }
}

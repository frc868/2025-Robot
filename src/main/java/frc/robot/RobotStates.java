package frc.robot;

import frc.robot.Constants.ReefLevel;

/** Global robot states shared across all subsystems. */
public class RobotStates {
    public static ReefLevel targetLevel = ReefLevel.L3;

    public static ReefLevel getTargetLevel() {
        targetLevel = ReefLevel.L2;
        return targetLevel;
    }

    public static void setTargetLevel(ReefLevel reefLevel) {
        targetLevel = reefLevel;
    }
}

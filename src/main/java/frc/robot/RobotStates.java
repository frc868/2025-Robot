package frc.robot;

import frc.robot.Constants.ReefLevel;

/** Global robot states shared across all subsystems. */
public class RobotStates {
    public static ReefLevel targetLevel;

    public static ReefLevel getTargetLevel() {
        return targetLevel;
    }

    public static void setTargetLevel(ReefLevel reefLevel) {
        targetLevel = reefLevel;
    }
}

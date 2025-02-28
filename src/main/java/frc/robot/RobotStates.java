package frc.robot;

import frc.robot.Constants.Level;

/** Global robot states shared across all subsystems. */
public class RobotStates {
    public static Level targetLevel = Level.L1;

    public static Level getTargetLevel() {
        return targetLevel;
    }

    public static void setTargetLevel(Level level) {
        targetLevel = level;
    }
}

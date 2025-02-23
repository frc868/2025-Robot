package frc.robot;

import frc.robot.Constants.Mode;
import frc.robot.Constants.Level;

/** Global robot states shared across all subsystems. */
public class RobotStates {
    public static Mode mode;
    public static Level targetLevel = Level.L1;

    public static Mode getMode() {
        return mode;
    }

    public static void setMode(Mode targetMode) {
        mode = targetMode;
    }

    public static Level getTargetLevel() {
        return targetLevel;
    }

    public static void setTargetLevel(Level level) {
        targetLevel = level;
    }
}

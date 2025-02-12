package frc.utils;

import java.util.function.Supplier;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class GoalPositions {
    /*
     * list of positions we care about that need pivot+elevator+more:
     * CORAL:
     * - levels 1-4
     * ALGAE:
     * - processor 2
     * - barge 5
     * - lower reef algae 3
     * - upper reef algae 4
     * - ground 1
     */
    // what the hell am i doing
    public static Supplier<Pivot.Constants.Position> pivotLocation(int level, boolean algae) {
        if (!algae) {
            switch (level) {
                case 1:
                    return Pivot.Constants.Position.L1;
                    break;
                case 2:
                    return Pivot.Constants.Position.L2;
                    break;
                case 3:
                    return Pivot.Constants.Position.L3;
                    break;
                case 4:
                    return Pivot.Constants.Position.L4;
                    break;
            }
        } else if (algae) {
            switch (level) {
                case 1:
                    return () -> Pivot.Constants.Position.GROUND;
                    break;
                case 2:
                    return () -> Pivot.Constants.Position.PROCESSOR;
                    break;
                case 3:
                    return () -> Pivot.Constants.Position.REEF_ALGAE_LOW;
                    break;
                case 4:
                    return () -> Pivot.Constants.Position.REEF_ALGAE_HIGH;
                    break;
                case 5:
                    return () -> Pivot.Constants.Position.NET;
                    break;
            }
        }
    }

    public static Supplier<Elevator.Constants.Position> elevatorLocation(int level, boolean algae) {
        if (!algae) {
            switch (level) {
                case 1:
                    return () -> Elevator.Constants.Position.L1;
                    break;
                case 2:
                    return () -> Elevator.Constants.Position.L2;
                    break;
                case 3:
                    return () -> Elevator.Constants.Position.L3;
                    break;
                case 4:
                    return () -> Elevator.Constants.Position.L4;
                    break;
            }
        } else if (algae) {
            switch (level) {
                case 1:
                    return () -> Elevator.Constants.Position.GROUND;
                    break;
                case 2:
                    return () -> Elevator.Constants.Position.PROCESSOR;
                    break;
                case 3:
                    return () -> Elevator.Constants.Position.REEF_ALGAE_LOW;
                    break;
                case 4:
                    return () -> Elevator.Constants.Position.REEF_ALGAE_HIGH;
                    break;
                case 5:
                    return () -> Elevator.Constants.Position.NET;
                    break;
            }
        }
    }
}

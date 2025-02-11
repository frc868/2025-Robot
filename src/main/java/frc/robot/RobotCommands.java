package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Vision;
import frc.utils.GoalPositions;

/**
 * Robot commands which involve multiple subsystems.
 * 
 * 
 * 
 * List of commands needed:
 * climbing
 * - setup climb, run climb X,X
 * align to reef branch X
 * - VISION, move and rotate to target X
 * intake algae from reef
 * - move to position X
 * ground intake algae
 * - setup intake, run intake motors X,X
 * move manipulator to scoring position
 * - move to position X
 * toggle mode
 * - toggle mode
 * lock to target
 * - VISION, rotate to target X
 * score
 * - run manip motors 1/2
 */
public class RobotCommands {

    public static Command prepareClimbCommand(Intake intake, Elevator elevator, Pivot pivot, Climber climber) {
        /** move pivot elevator and everything into position */
        return Commands.parallel(intake.moveToPositionCommand(() -> Intake.Constants.Position.SOME_CONSTANT),
                elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT),
                pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                climber.moveToPositionCommand(() -> Climber.Constants.Position.SOME_CONSTANT)); // TODO need real
                                                                                                // constants
    }

    public static Command climbCommand(Climber climber) { // TODO probably useless?
        /** move the thing down until its all the way down */
        return climber.moveToPositionCommand(() -> Climber.Constants.Position.SOME_CONSTANT); // TODO need real
                                                                                              // constant
    }

    // TODO add sideSupplier or whatever
    public static Command alignToBranchCommand(Drivetrain drivetrain) { // TODO this probably belongs in drivetrain

        /**
         * 1. locate april tag on reef
         * 2. move to certain x y offset
         */
        return drivetrain.driveToPoseCommand(drivetrain::chooseTargetBranch); // TODO fix it

    }

    /*
     * list of positions we care about that need pivot+elevator+more:
     * CORAL:
     * - levels 1-4
     * ALGAE:
     * - processor
     * - barge
     * - lower reef algae
     * - upper reef algae
     * - ground
     */

    public static Command intakeAlgaeReefCommand(Pivot pivot, Manipulator manipulator, Elevator elevator, int level) {
        /**
         * use vision to align to the reef (probably in a separate command?) then move
         * the manipulator into position then run it
         * prepare position first, then move it in and then grab
         */

        return Commands.sequence( // TODO real constants
                Commands.parallel(pivot.moveToPositionCommand(GoalPositions.pivotLocation(level, true)),
                        elevator.moveToPositionCommand(GoalPositions.elevatorLocation(level, true))),
                pivot.moveToPositionCommand(Pivot.Constants.Position.SOME_CONSTANT),
                manipulator.intakeGamePieceCommand());
    }

    public static Command intakeAlgaeGroundCommand(Pivot pivot, Manipulator manipulator, Elevator elevator,
            Intake intake) {
        /**
         * put the intake down then move the manipulator into position then run both the
         * intake and manipulator
         */
        return Commands.sequence(
                Commands.parallel(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT)),
                intake.moveToPositionCommand(Intake.Constants.Position.SOME_CONSTANT),
                Commands.parallel(intake.runRollersCommand(),
                        manipulator.intakeGamePieceCommand()));
    }

    public static Command ejectAlgaeGroundCommand(Pivot pivot, Manipulator manipulator, Elevator elevator,
            Intake intake) {
        /**
         * put the intake down then move the manipulator into position then run both the
         * intake and manipulator
         */
        return Commands.sequence(
                Commands.parallel(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT)),
                intake.moveToPositionCommand(Intake.Constants.Position.SOME_CONSTANT),
                Commands.parallel(intake.reverseRollersCommand(), // TODO iffy
                        manipulator.reverseRollersCommand()));
    }

    public static Command moveToAlgaeScoringPositionCommand(Pivot pivot, Elevator elevator, int level) {
        /**
         * move elevator and pivot to the right position
         */
        return Commands.parallel(pivot.moveToPositionCommand(GoalPositions.pivotLocation(level, true)),
                elevator.moveToPositionCommand(GoalPositions.elevatorLocation(level, true)));
    }

    public static Command moveToCoralScoringPositionCommand(Pivot pivot, Elevator elevator, int level) {
        /**
         * move elevator and pivot to the right position
         */
        return Commands.parallel(pivot.moveToPositionCommand(GoalPositions.pivotLocation(level, false)),
                elevator.moveToPositionCommand(GoalPositions.elevatorLocation(level, false)));
    }

    public static Command scoreAlgaeCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) { // TODO
                                                                                                       // parameters
        /**
         * just run the manipulator rollers (fast enough to throw the algae or slow
         * enough to be precise at the processor)
         */
        return manipulator.reverseRollersCommand(); // TODO specific voltage?
    }

    public static Command scoreCoralCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) {
        /**
         * reverse the rollers?
         */
        return manipulator.reverseRollersCommand(); // TODO specific voltage?
    }

    public static Command lockOnCommand(Drivetrain drivetrain, boolean reef) { /**
                                                                                * make it more descriptive
                                                                                */

        /**
         * constantly target reef with targetpose
         * 
         * () -> DriverStation.getAlliance().isPresent()
         * && DriverStation.getAlliance().get() == Alliance.Red
         * ? Reflector.reflectPose3d(FieldConstants.SPEAKER_TARGET,
         * FieldConstants.FIELD_LENGTH)
         * : FieldConstants.SPEAKER_TARGET)
         */
        return drivetrain
                .targetPoseCommand(() -> reef ? FieldConstants.TEMP_REEF_TARGET : FieldConstants.TEMP_BARGE_TARGET); // TODO
        // make
        // it
        // real
        // 'scoreCoralCommand'");
    }

}

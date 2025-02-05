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
        // throw new UnsupportedOperationException("Unimplemented command
        // 'climbCommand'");
    }

    public static Command climbCommand(Climber climber) { // TODO probably useless?
        /** move the thing down until its all the way down */
        return climber.moveToPositionCommand(() -> Climber.Constants.Position.SOME_CONSTANT); // TODO need real
                                                                                              // constant
        // throw new UnsupportedOperationException("Unimplemented command
        // 'climbCommand'");
    }

    // TODO add sideSupplier or whatever
    public static Command alignToBranchCommand(Drivetrain drivetrain) { // TODO this probably belongs in drivetrain

        /**
         * PROBABLY ONLY WORKS WHEN YOURE REALLY CLOSE TO ONE SPECIFIC APRILTAG
         * 1. gets current pose
         * - getPose()
         * 2. figures out which side of which reef we're on based on the april
         * tag/closest side
         * - for closest side: find i that gives min(getPose-apriltag_i) for i in {reef
         * apriltag #s}
         * 3. sets a target to move to
         * - thats gonna be pose2d with specific perp. theta for the side, then x,y is
         * pos of april tag + perp. vector pointing away a certain distance
         * - w/ knowledge of field move to certain position
         * 4. moves there
         * - driveToPose (?)
         */
        return drivetrain.driveToPoseCommand(drivetrain::chooseTargetBranch); // TODO fix it

    }

    public static Command intakeAlgaeReefCommand(Pivot pivot, Manipulator manipulator, Elevator elevator,
            Vision Vision, Drivetrain drivetrain) {
        /**
         * use vision to align to the reef (probably in a separate command?) then move
         * the manipulator into position then run it
         * prepare position first, then move it in and then grab
         */
        return Commands.sequence( // TODO real constants
                Commands.parallel(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT)),
                pivot.moveToPositionCommand(Pivot.Constants.Position.SOME_CONSTANT),
                manipulator.intakeGamePieceCommand());
        // throw new UnsupportedOperationException("Unimplemented command
        // 'intakeAlgaeReefCommand'");
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
        // throw new UnsupportedOperationException("Unimplemented command
        // 'intakeAlgaeGroundCommand'");
    }

    // TODO positionSupplier something
    public static Command moveToAlgaeScoringPositionCommand(Pivot pivot, Elevator elevator) { /**
                                                                                               * maybe a parameter for
                                                                                               * position
                                                                                               */
        /**
         * move elevator and pivot to the right position
         */
        return Commands.sequence(
                Commands.parallel(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT)));
    }

    public static Command moveToCoralScoringPositionCommand(Pivot pivot, Elevator elevator) { /**
                                                                                               * maybe a parameter for
                                                                                               * position
                                                                                               */
        /**
         * move elevator then pivot to the right position
         */
        return Commands.sequence(
                Commands.parallel(pivot.moveToPositionCommand(() -> Pivot.Constants.Position.SOME_CONSTANT),
                        elevator.moveToPositionCommand(() -> Elevator.Constants.Position.SOME_CONSTANT)));
        throw new UnsupportedOperationException("Unimplemented command 'moveToCoralScoringPositionCommand'");
    }

    public static Command scoreAlgaeCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) { // TODO
                                                                                                       // parameters
        /**
         * just run the manipulator rollers (fast enough to throw the algae or slow
         * enough to be precise at the processor)
         */
        return manipulator.reverseRollersCommand(); // TODO specific voltage?
        // throw new UnsupportedOperationException("Unimplemented command
        // 'scoreAlgaeCommand'");
    }

    public static Command scoreCoralCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) { /**
                                                                                                        * maybe a
                                                                                                        * parameter for
                                                                                                        * position
                                                                                                        */
        /**
         * 
         */
        throw new UnsupportedOperationException("Unimplemented command 'scoreCoralCommand'");
    }

    public static Command lockOnCommand(Drivetrain drivetrain) { /**
                                                                  * parameter for
                                                                  * position
                                                                  * reef or barge or processor
                                                                  */

        /**
         * constantly target reef with targetpose
         */
        return drivetrain.targetPoseCommand(null); // TODO way to calculate target
        // throw new UnsupportedOperationException("Unimplemented command
        // 'scoreCoralCommand'");
    }

}

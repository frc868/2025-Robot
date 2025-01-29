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

/** Robot commands which involve multiple subsystems. */
public class RobotCommands {

    public static Command climbCommand(Climber climber) {
        /** move the thing down until its all the way down */
        throw new UnsupportedOperationException("Unimplemented command 'climbCommand'");
    }

    public static Command intakeAlgaeReefCommand(Pivot pivot, Manipulator manipulator, Elevator elevator,
            Vision Vision, Drivetrain drivetrain) {
        /**
         * use vision to align to the reef (maybe in a separate command?) then move the
         * manipulator into position then run it
         */
        throw new UnsupportedOperationException("Unimplemented command 'intakeAlgaeReefCommand'");
    }

    public static Command intakeAlgaeGroundCommand(Pivot pivot, Manipulator manipulator, Elevator elevator,
            Intake intake) {
        /**
         * put the intake down then move the manipulator into position then run both the
         * intake and manipulator
         */
        throw new UnsupportedOperationException("Unimplemented command 'intakeAlgaeGroundCommand'");
    }

    public static Command moveToAlgaeScoringPositionCommand(Pivot pivot, Elevator elevator) { /**
                                                                                               * maybe a parameter for
                                                                                               * position
                                                                                               */
        /**
         * move elevator then pivot to the right position
         */
        throw new UnsupportedOperationException("Unimplemented command 'moveToAlgaeScoringPositionCommand'");
    }

    public static Command moveToCoralScoringPositionCommand(Pivot pivot, Elevator elevator) { /**
                                                                                               * maybe a parameter for
                                                                                               * position
                                                                                               */
        /**
         * move elevator then pivot to the right position
         */
        throw new UnsupportedOperationException("Unimplemented command 'moveToCoralScoringPositionCommand'");
    }

    public static Command scoreAlgaeCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) { /**
                                                                                                        * maybe a
                                                                                                        * parameter for
                                                                                                        * position
                                                                                                        */
        /**
         * move the manipulator the right way
         */
        throw new UnsupportedOperationException("Unimplemented command 'scoreAlgaeCommand'");
    }

    public static Command scoreCoralCommand(Pivot pivot, Elevator elevator, Manipulator manipulator) { /**
                                                                                                        * maybe a
                                                                                                        * parameter for
                                                                                                        * position
                                                                                                        */
        /**
         * move the manipulator the right way (maybe scoring coral will require
         * rotations of the pivot)
         */
        throw new UnsupportedOperationException("Unimplemented command 'scoreCoralCommand'");
    }

}

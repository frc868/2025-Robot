package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Pivot;

public class HoundBrian {
    private final DigitalInput drivetrainButton = new DigitalInput(0);
    private final DigitalInput elevatorButton = new DigitalInput(1);
    private final DigitalInput pivotButton = new DigitalInput(2);
    private final DigitalInput actionButton = new DigitalInput(3);
    private final DigitalInput actionButton2 = new DigitalInput(4);

    public HoundBrian(Drivetrain drivetrain, Elevator elevator, Pivot pivot, LEDs leds) {
        new Trigger(drivetrainButton::get).negate().and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));

        new Trigger(elevatorButton::get).negate().and(DriverStation::isDisabled)
                .onTrue(elevator.resetPositionCommand().ignoringDisable(true));

        new Trigger(pivotButton::get).negate().and(DriverStation::isDisabled)
                .onTrue(pivot.resetPositionCommand().ignoringDisable(true));

        // Simply to test if LED patterns actual work
        new Trigger(actionButton::get)
                .whileTrue(leds.requestStateCommand(LEDState.SOLID_GREEN).ignoringDisable(true));
        new Trigger(actionButton2::get)
                .whileTrue(leds.requestStateCommand(LEDState.SOLID_GREEN).ignoringDisable(true));
    }
}

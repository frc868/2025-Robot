// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs;

public class RobotContainer {

    private final LEDs leds = new LEDs();

    // Add all subsystems later
    @Log(groups = "subsystems")
    private final HoundBrian HoundBrian = new HoundBrian(leds);

    public RobotContainer() {
        configureBindings();
        configureHoundBrianResponses();
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public LEDs getLEDs() {
        return leds;
    }

    private void configureHoundBrianResponses() {
        new Trigger(houndBrian::isZeroingActivated)
                .onActive(() -> {
                    leds.requestStateCommand(LEDs.LEDState.SOLID_GREEN).withTimeout(3); // Green for 3 seconds
                    resetAllSubsystems();
                });

        new Trigger(houndBrian::isZeroingActivated)
                .onActive(() -> leds.requestStateCommand(LEDs.LEDState.INITIALIZED_CONFIRM).schedule());

        new Trigger(houndBrian::isFaultDetected)
                .onActive(() -> leds.requestStateCommand(LEDs.LEDState.FLASHING_RED).schedule());
    }

    private void resetAllSubsystems() {
        // reset
    }

}

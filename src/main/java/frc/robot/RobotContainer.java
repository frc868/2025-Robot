// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LEDs;

public class RobotContainer {

    private final LEDs leds = new LEDs();

    // Add all subsystems later
    @Log(groups = "subsystems")
    private final HoundBrian houndBrian = new HoundBrian(drivetrain, intake, leds);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // HoundBrian handles the individual bindings internally, but here we check all
        // together.
        new Trigger(() -> houndBrian.drivetrainButton.get() &&
                houndBrian.intakeButton.get() &&
                houndBrian.shooterTiltButton.get() &&
                houndBrian.climberButton.get() &&
                DriverStation.getInstance().isDisabled()).onActive(() -> {
                    leds.requestStateCommand(LEDs.LEDState.SOLID_GREEN).withTimeout(3); // Green for 3 seconds
                    resetAllSubsystems();
                }).onInactive(() -> {
                    leds.requestStateCommand(LEDs.LEDState.RAINBOW_WAVE); // Example of default state
                });
    }

    public LEDs getLEDs() {
        return leds;
    }

    private void resetAllSubsystems() {
        // reset
    }

}

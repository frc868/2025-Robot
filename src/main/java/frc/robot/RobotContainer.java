// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class RobotContainer {

    @Log
    private final LEDs leds = new LEDs();

    @Log
    Drivetrain drivetrain = new Drivetrain();

    @Log
    Intake intake = new Intake();

    // Keep adding more
    @Log
    HoundBrian houndBrian = new HoundBrian(drivetrain, intake, leds);

    public RobotContainer() {
        configureBindings();
    }

    public void configureBindings() {
        Controls.configureBindings
    }
}

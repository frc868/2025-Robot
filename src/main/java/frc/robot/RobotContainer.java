// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;

import frc.robot.subsystems.LEDs;

public class RobotContainer {

    @Log(groups = "subsystems")
    private final LEDs leds = new LEDs();

    public RobotContainer() {
    }
}

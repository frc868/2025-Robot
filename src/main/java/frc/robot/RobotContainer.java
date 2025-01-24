// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    @Log(groups = "subsystems")
    private final Drivetrain drivetrain = new Drivetrain();

    @SendableLog(groups = "wpilib")
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    private double prevLoopTime = 0.0;

    @Log(groups = "wpilib")
    private final Supplier<Double> loopTimeMs = () -> {
        double timestamp = Timer.getFPGATimestamp();
        double loopTime = Timer.getFPGATimestamp() - prevLoopTime;
        prevLoopTime = timestamp;
        return loopTime * 1000.0;
    };

    @Log(groups = "wpilib")
    private final Supplier<Double> matchTimer = DriverStation::getMatchTime;

    public RobotContainer() {
        configureBindings();

        // new Trigger(() -> {
        // return drivetrain.getInitialized();
        // }).onTrue(GlobalStates.INITIALIZED.enableCommand());
    }

    private void configureBindings() {
        Controls.configureDriverControls(0, drivetrain);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
    @Log(groups = "subsystems")
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Pivot pivot = new Pivot();
    private final Manipulator manipulator = new Manipulator();
    private final Climber climber = new Climber();
    private final LEDs leds = new LEDs();

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
        Controls.configureDriverControls(0, drivetrain, elevator, pivot, manipulator, intake, climber, leds);
        Controls.configureOperatorControls(1, drivetrain, elevator, pivot, manipulator, intake, climber, leds);
        Controls.configureOverrideControls(2, drivetrain, elevator, pivot, manipulator, intake, climber, leds);
        Controls.configureSysIdControls(3, drivetrain, elevator, pivot, manipulator, intake, climber, leds);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

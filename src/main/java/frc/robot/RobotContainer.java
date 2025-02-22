// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Level;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Pivot;

@Logged
public class RobotContainer {
    private final PositionTracker positionTracker = new PositionTracker();

    @Log(groups = "subsystems")
    private final Drivetrain drivetrain = new Drivetrain();

    @Log(groups = "subsystems")
    private final Intake intake = new Intake();

    @Log(groups = "subsystems")
    private final Elevator elevator = new Elevator(positionTracker);

    @Log(groups = "subsystems")
    private final Pivot pivot = new Pivot(positionTracker);

    @Log(groups = "subsystems")
    private final Manipulator manipulator = new Manipulator();

    @Log(groups = "subsystems")
    private final Climber climber = new Climber();

    // @Log(groups = "subsystems")
    // private final LEDs leds = new LEDs();

    @Log(groups = "subsystems")
    private final HoundBrian houndBrian = new HoundBrian(drivetrain, elevator, pivot);

    @SendableLog(groups = "wpilib")
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    @Log
    private Supplier<Pose3d[]> componentPoses = () -> new Pose3d[] {
            elevator.getStageComponentPose(),
            elevator.getCarriageComponentPose(),
            pivot.getComponentPose(),
            intake.getLeftFrontBarComponentPose(),
            intake.getRightFrontBarComponentPose(),
            intake.getTopBarComponentPose(),
            intake.getLeftBackBarComponentPose(),
            intake.getRightBackBarComponentPose(),
            climber.getComponentPose()
    };
    @Log
    private Supplier<Pose3d> coralPose = () -> new Pose3d(drivetrain.getSimPose()).plus(pivot.getCoralTransform());
    @Log
    private Supplier<Pose3d> algaePose = () -> new Pose3d(drivetrain.getSimPose()).plus(pivot.getAlgaeTransform());

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
        configureAuto();

        LoggingManager.getInstance().registerObject(this);

        // new Trigger(() -> {
        // return drivetrain.getInitialized();
        // }).onTrue(GlobalStates.INITIALIZED.enableCommand());
    }

    private void configureBindings() {
        Controls.configureDriverControls(0, drivetrain, elevator, pivot, manipulator, intake, climber);
        Controls.configureOperatorControls(1, drivetrain, elevator, pivot, manipulator, intake, climber);
        Controls.configureOverrideControls(2, drivetrain, elevator, pivot, manipulator, intake, climber);
        Controls.configureSysIdControls(3, drivetrain, elevator, pivot, manipulator, intake, climber);
    }

    private void configureAuto() {
        // AutoManager.getInstance().addRoutine(Autos.auto(drivetrain, elevator,
        // pivot, manipulator, intake, climber));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.PositionTracker;
import com.techhounds.houndutil.houndlog.LoggingManager;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.SendableLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants.Reef.ReefLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDState;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private Mechanism2d mechanisms = new Mechanism2d(5, 3);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);

    @SuppressWarnings("unused")
    private MechanismLigament2d fromRobot = root
            .append(new MechanismLigament2d("fromRobot", Units.inchesToMeters(5.5), 180, 0,
                    new Color8Bit(Color.kWhite)));
    @SuppressWarnings("unused")
    private MechanismLigament2d elevatorBase = root
            .append(new MechanismLigament2d("elevatorBase", Units.inchesToMeters(36), 90, 2,
                    new Color8Bit(Color.kWhite)));
    private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90,
                    4,
                    new Color8Bit(Color.kOrange)));
    private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));

    PositionTracker positionTracker = new PositionTracker();

    @Log(groups = "subsystems")
    Drivetrain drivetrain = new Drivetrain(positionTracker);
    @Log(groups = "subsystems")
    Elevator elevator = new Elevator(positionTracker, elevatorLigament);
    @Log(groups = "subsystems")
    Arm arm = new Arm(positionTracker, armLigament);
    @Log(groups = "subsystems")
    Manipulator manipulator = new Manipulator(elevator.isStowed);
    @Log(groups = "subsystems")
    Climber climber = new Climber(positionTracker);
    @Log(groups = "subsystems")
    Vision vision = new Vision();
    @Log(groups = "subsystems")
    LEDs leds = new LEDs(drivetrain);

    @Log
    Superstructure superstructure = new Superstructure(drivetrain, elevator, arm, manipulator, climber, leds);

    @Log(groups = "subsystems")
    HoundBrian houndbrian = new HoundBrian(drivetrain, elevator, arm, climber, leds);

    @Log
    private final Supplier<Boolean> initialized = GlobalStates.INITIALIZED::enabled;

    @Log(groups = "components")
    private Supplier<Pose3d> coralPose = () -> manipulator.hasCoral.getAsBoolean()
            ? new Pose3d(drivetrain.getPrecisePose()).plus(arm.getCoralTransform())
            : new Pose3d(-1000, -1000, -1000, new Rotation3d());
    @Log(groups = "components")
    private Supplier<Pose3d> algaePose = () -> manipulator.hasAlgae.getAsBoolean()
            ? new Pose3d(drivetrain.getPrecisePose()).plus(arm.getAlgaeTransform())
            : new Pose3d(-1000, -1000, -1000, new Rotation3d());

    @Log(groups = "components")
    private Supplier<Pose3d[]> componentPoses = () -> new Pose3d[] {
            elevator.getStageComponentPose(),
            elevator.getCarriageComponentPose(),
            arm.getComponentPose(),
            climber.getComponentPose()
    };

    @SendableLog
    CommandScheduler scheduler = CommandScheduler.getInstance();

    @Log(groups = "timing")
    private Supplier<Double> odometryLoopTimeMs = () -> drivetrain.getOdometryLoopTime();

    @Log
    private final Supplier<Double> matchTimer = DriverStation::getMatchTime;

    public RobotContainer() {
        configureBindings();
        configureAuto();
        configureTriggers();

        LoggingManager.getInstance().registerObject(this);
        LoggingManager.getInstance().registerMetadata(Constants.BUILD_METADATA);
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();
        SignalLogger.enableAutoLogging(false);

        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        vision.setSimPoseSupplier(drivetrain::getSimPose);
        vision.setPoseEstimator(drivetrain.getPoseEstimator());
        vision.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds);
        vision.setHeadingSupplier(drivetrain::getRotation);
        vision.setVisionMeasurementConsumer(drivetrain::addVisionMeasurement);
        vision.setPreciseVisionMeasurementConsumer(drivetrain::addPreciseVisionMeasurement);
    }

    private void configureBindings() {
        Controls.configureControls(0, drivetrain, superstructure);
        Controls.configureTestingControls(1, drivetrain, superstructure);
    }

    private void configureTriggers() {
        new Trigger(() -> {
            return drivetrain.getInitialized()
                    && elevator.getInitialized()
                    && arm.getInitialized()
                    && climber.getInitialized();
        }).onTrue(GlobalStates.INITIALIZED.enableCommand());

        new Trigger(GlobalStates.INITIALIZED::enabled)
                .onTrue(leds.requestStateCommand(LEDState.INITIALIZED_CONFIRM).withTimeout(3));

        leds.requestStateCommand(LEDState.INITIALIZATION_BLACK_BACKGROUND).until(GlobalStates.INITIALIZED::enabled)
                .schedule();
        leds.requestStateCommand(LEDState.DRIVETRAIN_UNINITIALIZED).until(drivetrain::getInitialized).schedule();
        leds.requestStateCommand(LEDState.ELEVATOR_UNINITIALIZED).until(elevator::getInitialized).schedule();
        leds.requestStateCommand(LEDState.ARM_UNINITIALIZED).until(arm::getInitialized).schedule();
        leds.requestStateCommand(LEDState.CLIMBER_UNINITIALIZED).until(climber::getInitialized).schedule();

        new Trigger(DriverStation::isEnabled)
                .onTrue(Commands.parallel(
                        elevator.resetControllersCommand(),
                        arm.resetControllersCommand()).withName("resetControllers"));

        elevator.isStowed.and(manipulator.hasCoral.negate()).and(RobotModeTriggers.disabled().negate())
                .whileTrue(leds.requestStateCommand(LEDState.READY_FOR_CORAL));
        manipulator.hasCoral.and(RobotModeTriggers.disabled().negate())
                .whileTrue(leds.requestStateCommand(LEDState.HAS_CORAL));
        manipulator.hasAlgae.and(RobotModeTriggers.disabled().negate())
                .whileTrue(leds.requestStateCommand(LEDState.HAS_ALGAE));

        manipulator.hasCoral.whileTrue(arm.useCoralGainsCommand());
        manipulator.hasAlgae.whileTrue(arm.useAlgaeGainsCommand());

        superstructure.isReadyToScore
                .whileTrue(leds.requestStateCommand(LEDState.READY_TO_SCORE));

        new Trigger(() -> superstructure.getReefSetpoint().orElse(null) == ReefLevel.L1)
                .whileTrue(leds.requestStateCommand(LEDState.L1));
        new Trigger(() -> superstructure.getReefSetpoint().orElse(null) == ReefLevel.L2)
                .whileTrue(leds.requestStateCommand(LEDState.L2));
        new Trigger(() -> superstructure.getReefSetpoint().orElse(null) == ReefLevel.L3)
                .whileTrue(leds.requestStateCommand(LEDState.L3));
        new Trigger(() -> superstructure.getReefSetpoint().orElse(null) == ReefLevel.L4)
                .whileTrue(leds.requestStateCommand(LEDState.L4));

        new Trigger(() -> GlobalStates.LOCK_TO_LINE_OVERRIDE.enabled())
                .whileTrue(leds.requestStateCommand(LEDState.LOCK_TO_LINE_OVERRIDE));
        new Trigger(() -> GlobalStates.MANUAL_DRIVING_OVERRIDE.enabled())
                .whileTrue(leds.requestStateCommand(LEDState.MANUAL_DRIVING_OVERRIDE));
    }

    public void configureAuto() {
        try {
            AutoManager.getInstance().addRoutine(Autos.FDCB(drivetrain, superstructure));
            AutoManager.getInstance().addRoutine(Autos.FDCE(drivetrain, superstructure));
            // AutoManager.getInstance().addRoutine(Autos.FDCBExperimental(drivetrain,
            // superstructure));
            // AutoManager.getInstance().addRoutine(Autos.FBDC(drivetrain, superstructure));
            // AutoManager.getInstance().addRoutine(Autos.IKL(drivetrain, superstructure));
            AutoManager.getInstance().addRoutine(Autos.IKLA(drivetrain, superstructure));
            AutoManager.getInstance().addRoutine(Autos.IKLJ(drivetrain, superstructure));
            AutoManager.getInstance().addRoutine(Autos.wheelRadiusCharacterization(drivetrain));
            AutoManager.getInstance().addRoutine(Autos.HAlgae(drivetrain, superstructure));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Could not create trajectories.");
        }
    }
}

package frc.robot.subsystems;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseSwerveDrive;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Subsystem which drives robot with swerve drive. */
public class Drivetrain extends SubsystemBase implements BaseSwerveDrive {
    /** Constant values of drivetrain subsystem. */
    public static final class Constants {
    }

    @Override
    public DriveMode getDriveMode() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveMode'");
    }

    @Override
    public Pose2d getPose() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPose'");
    }

    @Override
    public Rotation2d getRotation() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation'");
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModulePositions'");
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModuleStates'");
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeeds'");
    }

    @Override
    public SwerveDrivePoseEstimator getPoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPoseEstimator'");
    }

    @Override
    public void updatePoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstimator'");
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPoseEstimator'");
    }

    @Override
    public void resetGyro() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetGyro'");
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotorHoldModes'");
    }

    @Override
    public void setDriveCurrentLimit(int currentLimit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveCurrentLimit'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void setStates(SwerveModuleState[] state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStates'");
    }

    @Override
    public void setStatesClosedLoop(SwerveModuleState[] state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatesClosedLoop'");
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public void drive(ChassisSpeeds speeds, DriveMode driveMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds, DriveMode driveMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveClosedLoop'");
    }

    @Override
    public Command teleopDriveCommand(
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier thetaSpeedSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopDriveCommand'");
    }

    @Override
    public Command controlledRotateCommand(DoubleSupplier angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'controlledRotateCommand'");
    }

    @Override
    public Command disableControlledRotateCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
                "Unimplemented method 'disableControlledRotateCommand'");
    }

    @Override
    public Command wheelLockCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'wheelLockCommand'");
    }

    @Override
    public Command turnWheelsToAngleCommand(double angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'turnWheelsToAngleCommand'");
    }

    @Override
    public Command driveToPoseCommand(Supplier<Pose2d> pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveToPoseCommand'");
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'followPathCommand'");
    }

    @Override
    public Command driveDeltaCommand(Transform2d delta, PathConstraints constraints) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'driveDeltaCommand'");
    }

    @Override
    public Command setDriveModeCommand(DriveMode driveMode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveModeCommand'");
    }

    @Override
    public Command resetGyroCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetGyroCommand'");
    }

    @Override
    public Command setDriveCurrentLimitCommand(int currentLimit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDriveCurrentLimitCommand'");
    }

    @Override
    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }
}

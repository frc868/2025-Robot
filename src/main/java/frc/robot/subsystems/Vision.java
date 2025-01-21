package frc.robot.subsystems;

import com.techhounds.houndutil.houndlib.subsystems.BaseVision;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

/** Subsystem which detecs AprilTags and scoring elements. */
public class Vision implements BaseVision {
    public static final class Constants {
    }

    @Override
    public void updatePoseEstimator() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePoseEstimator'");
    }

    @Override
    public Pose3d[] getCameraPoses() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCameraPoses'");
    }

    @Override
    public Pose3d[] getAprilTagPoses() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAprilTagPoses'");
    }

    @Override
    public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPoseEstimator'");
    }

    @Override
    public void setSimPoseSupplier(Supplier<Pose2d> simPoseSupplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSimPoseSupplier'");
    }
}

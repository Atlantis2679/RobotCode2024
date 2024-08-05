package frc.robot.subsystems.swerve.poseEstimator;
import frc.lib.logfields.LogFieldsTable;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class VisionAprilTagsIOLimelight extends VisionAprilTagsIO  {

    private static LimelightHelpers.PoseEstimate limelightResults;
        public VisionAprilTagsIOLimelight(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void periodicBeforeFields() {

        limelightResults = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    }

    @Override
    protected double getRobotPoseTimestampSeconds() {
        return limelightResults.timestampSeconds;

    }

    @Override
    protected Pose3d getRobotPose() {
        Pose3d estimate = new Pose3d(limelightResults.pose);
        return estimate != null ? estimate : new Pose3d();
    }

    @Override
    protected boolean getHasNewRobotPose() {
        return limelightResults.tagCount > 0;
    }
}


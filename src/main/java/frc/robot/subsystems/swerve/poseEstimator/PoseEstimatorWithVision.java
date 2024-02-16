package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;

import java.io.IOException;

import org.photonvision.PhotonUtils;

public class PoseEstimatorWithVision {
    private final VisionAprilTagsIO visionIO;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final LogFieldsTable fieldsTable;

    public PoseEstimatorWithVision(LogFieldsTable fieldsTable, Rotation2d currentAngle,
            SwerveModulePosition[] positions, SwerveDriveKinematics swerveKinematics) {
        try {
            AprilTagFieldLayout tagsLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            visionIO = Robot.isSimulation()
                    ? new VisionAprilTagsIOSim(fieldsTable)
                    : new VisionAprilTagsIOPhoton(fieldsTable, tagsLayout);
        } catch (IOException e) {
            DriverStation.reportError("AprilTagFieldLayout blew up", e.getStackTrace());
            throw new RuntimeException(e);
        }
        this.fieldsTable = fieldsTable;

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                currentAngle,
                positions,
                new Pose2d());
    }

    public void update(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions) {
        poseEstimator.update(gyroMeasurmentCCW, modulesPositions);
        double visionToEstimateDifferenceMeters = getVisionToEstimateDifferenceMeters();
        fieldsTable.recordOutput("Vision To Estimate Difference", visionToEstimateDifferenceMeters);
        if (visionToEstimateDifferenceMeters < VisionAprilTagsConstants.VISION_THRESHOLD_DISTANCE_M
                && visionIO.getHasNewRobotPose()) {

            poseEstimator.addVisionMeasurement(visionIO.photonPoseEstimate.get().toPose2d(),
                    visionIO.cameraTimestampSeconds.getAsDouble());
        }
    }

    private double getVisionToEstimateDifferenceMeters() {
        return PhotonUtils.getDistanceToPose(visionIO.photonPoseEstimate.get().toPose2d(),
                poseEstimator.getEstimatedPosition());
    }

    public void resetPosition(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroMeasurmentCCW, modulesPositions, newPose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}

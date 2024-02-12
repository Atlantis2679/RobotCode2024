package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveContants;
import frc.robot.subsystems.swerve.VisionPoseEstimatorConstants;

import java.io.IOException;

import org.photonvision.PhotonUtils;

public class PoseEstimator {
    private final VisionPoseEstimatorIO visionIO;

    private final SwerveDrivePoseEstimator poseEstimator;

    public final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            -SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_LEFT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_WIDTH_M / 2,
            -SwerveContants.TRACK_LENGTH_M / 2);

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout;

    static{
        try {
            kTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("AprilTagFieldLayout blew up");
            throw new RuntimeException(e);
        }
    }

    public PoseEstimator(LogFieldsTable fieldsTable, Rotation2d currentAngle, SwerveModulePosition[] positions, SwerveDriveKinematics swerveKinematics) {
        visionIO = Robot.isSimulation()
                ? new VisionPoseEstimatorIOSim(fieldsTable, this::getPose, kTagLayout)
                : new VisionPoseEstimatorIOPhoton(fieldsTable, kTagLayout);

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                currentAngle,
                positions,
                new Pose2d());
    }
    
    public void updatePoseEstimator(Rotation2d currentAngleDegrees, SwerveModulePosition[] positions) {
        poseEstimator.update(currentAngleDegrees, positions);
    }

    public void addVisionMeasurements() {

        double poseEstimateDiff = this.getDistanceToPose();
            if(poseEstimateDiff < VisionPoseEstimatorConstants.VISION_THRESHOLD_DISTANCE && visionIO.cameraHasTarget()){
                
                poseEstimator.addVisionMeasurement(visionIO.photonPoseEstimate.get().toPose2d(), visionIO.cameraTimestampSeconds.getAsDouble());
            }
    }

    public double getDistanceToPose() {
        return PhotonUtils.getDistanceToPose(visionIO.photonPoseEstimate.get().toPose2d(), poseEstimator.getEstimatedPosition());
    }

    public void resetPosition(Rotation2d currentAngle, SwerveModulePosition[] positions, Pose2d currentPosition) {
        poseEstimator.resetPosition(currentAngle, positions, currentPosition);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPhotonPoseEstimator() {
        return visionIO.photonPoseEstimate.get().toPose2d();
    }
}

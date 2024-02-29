package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.math.geometry.Pose3d;

import frc.lib.logfields.LogFieldsTable;

public class VisionAprilTagsIOSim extends VisionAprilTagsIO {
    public VisionAprilTagsIOSim(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    @Override
    public void periodicBeforeFields(){
    }

    @Override
    protected double getRobotPoseTimestampSeconds(){
        return 0;
    }

    @Override
    protected Pose3d getRobotPose(){
        return new Pose3d();
    }

    @Override
    protected boolean getHasNewRobotPose(){
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.LogFieldsTable;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera camera;
    private PhotonPipelineResult photonPipelineResult;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, AprilTagFieldLayout tagLayout){
        super(fieldsTable);

        camera = new PhotonCamera("camera");
        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionAprilTagsConstants.ROBOT_TO_CAMERA_TRANSFORM);

    }

    @Override
    public void periodicBeforeFields(){
        photonPipelineResult = camera.getLatestResult();
    }

    @Override
    protected double getRobotPoseTimestampSeconds(){
        return photonPipelineResult.getTimestampSeconds();
    }

    @Override
    protected Pose3d getRobotPose(){
        return photonPoseEstimator.update(photonPipelineResult).get().estimatedPose;
    }

    @Override
    protected boolean getHasNewRobotPose(){
        return photonPipelineResult.hasTargets();
    }

}

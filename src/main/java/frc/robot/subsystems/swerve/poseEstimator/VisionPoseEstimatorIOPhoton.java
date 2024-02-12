// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;


import frc.robot.subsystems.swerve.VisionPoseEstimatorConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.LogFieldsTable;


/** Add your docs here. */
public class VisionPoseEstimatorIOPhoton extends VisionPoseEstimatorIO {

    // Construct PhotonPoseEstimator
    PhotonPoseEstimator photonPoseEstimator;
    private final LogFieldsTable fieldsTable;
    private PhotonCamera camera;
    private PhotonPipelineResult photonPipelineResult;
    public VisionPoseEstimatorIOPhoton(LogFieldsTable fieldsTable, AprilTagFieldLayout kTagLayout){
        super(fieldsTable);
        this.fieldsTable = fieldsTable;

        camera = new PhotonCamera("camera");
        photonPoseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionPoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM);

    }

    @Override
    public void periodicBeforeFields(){

        photonPipelineResult = camera.getLatestResult();
    }

    @Override
    protected double getCameraTimestampSeconds(){
        return photonPipelineResult.getTimestampSeconds();
    }

    @Override
    protected Pose3d getPoseEstimate(){
        return photonPoseEstimator.update(photonPipelineResult).get().estimatedPose;
    }

    @Override
    protected boolean cameraHasTarget(){
        return photonPipelineResult.hasTargets();
    }

}

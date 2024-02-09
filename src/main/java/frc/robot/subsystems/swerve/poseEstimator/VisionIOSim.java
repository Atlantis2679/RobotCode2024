// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import org.photonvision.PhotonCamera;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.logfields.LogFieldsTable;

import frc.robot.subsystems.swerve.VisionConstants;

/** Add your docs here. */
public class VisionIOSim extends VisionIO {
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final Supplier<Pose2d> estimatedPositionSupplier;
    private Pose3d simPoseEstimate;
    //private PhotonPoseEstimator simPhotonEstimator;
    private final LogFieldsTable fieldsTable;

    PhotonCamera camera = new PhotonCamera("camera");
    private double cameraLatestTimestamp;
    public VisionIOSim(LogFieldsTable fieldsTable, Supplier<Pose2d> estimatedPositionSupplier){
        super(fieldsTable);
        this.fieldsTable = fieldsTable;
        this.estimatedPositionSupplier = estimatedPositionSupplier;
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("vision simulation");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(VisionConstants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(VisionConstants.simCameraResWidth, VisionConstants.simCameraResHeight, Rotation2d.fromDegrees(VisionConstants.simCameraFovDeg));
        cameraProp.setCalibError(VisionConstants.simCameraAvgErrorPx, VisionConstants.simCameraErrorStdDevPx);
        cameraProp.setFPS(VisionConstants.simCameraFps);
        cameraProp.setAvgLatencyMs(VisionConstants.simCameraAvgLatencyMs);
        cameraProp.setLatencyStdDevMs(VisionConstants.simCameraLatencyStdDevMs);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, VisionConstants.robotToCameraTransform3d);
        visionSim.addAprilTags(VisionConstants.kTagLayout);
    }

    @Override
    public void periodicBeforeFields(){

        visionSim.update(estimatedPositionSupplier.get());
        
        PhotonPipelineResult photonPipelineResult = camera.getLatestResult();
        cameraLatestTimestamp = photonPipelineResult.getTimestampSeconds();
        simPoseEstimate = visionSim.getRobotPose(cameraLatestTimestamp);
 
        Logger.recordOutput("simulated camera pose", visionSim.getCameraPose(cameraSim).get().toPose2d());
    }

    @Override
    protected double getCameraTimestampSeconds(){
        return cameraLatestTimestamp;
    }

    @Override
    protected Pose3d getPoseEstimate(){
        return simPoseEstimate;
    }

}

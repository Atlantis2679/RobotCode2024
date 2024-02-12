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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.logfields.LogFieldsTable;

import frc.robot.subsystems.swerve.VisionPoseEstimatorConstants;

/** Add your docs here. */
public class VisionPoseEstimatorIOSim extends VisionPoseEstimatorIO {
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final Supplier<Pose2d> estimatedPositionSupplier;
    private Pose3d simPoseEstimate;
    //private PhotonPoseEstimator simPhotonEstimator;
    private final LogFieldsTable fieldsTable;

    PhotonCamera camera = new PhotonCamera("camera");
    private double cameraLatestTimestamp;

    public VisionPoseEstimatorIOSim(LogFieldsTable fieldsTable, Supplier<Pose2d> estimatedPositionSupplier, AprilTagFieldLayout kTagLayout){
        super(fieldsTable);
        this.fieldsTable = fieldsTable;
        this.estimatedPositionSupplier = estimatedPositionSupplier;
        // Create the vision system simulation which handles cameras and targets on the field.
        visionSim = new VisionSystemSim("vision simulation");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        visionSim.addAprilTags(kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(VisionPoseEstimatorConstants.SIM_CAMERA_RES_WIDTH, VisionPoseEstimatorConstants.SIM_CAMERA_RES_HEIGHT, Rotation2d.fromDegrees(VisionPoseEstimatorConstants.SIM_CAMERA_FOV_DEG));
        cameraProp.setCalibError(VisionPoseEstimatorConstants.SIM_CAMERA_AVG_ERROR_PX, VisionPoseEstimatorConstants.SIM_CAMERA_ERROR_STD_DEV_PX);
        cameraProp.setFPS(VisionPoseEstimatorConstants.SIM_CAMERA_FPS);
        cameraProp.setAvgLatencyMs(VisionPoseEstimatorConstants.SIM_CAMERA_AVG_LATENCY_MS);
        cameraProp.setLatencyStdDevMs(VisionPoseEstimatorConstants.SIM_CAMERA_LATENCY_STD_DEV_MS);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(cameraSim, VisionPoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM);
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

    @Override
    protected boolean cameraHasTarget(){
        return false; //-----not-using-simulation------
    }

}

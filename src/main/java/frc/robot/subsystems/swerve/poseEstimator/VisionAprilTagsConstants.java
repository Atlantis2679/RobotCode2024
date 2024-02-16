// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionAprilTagsConstants {
    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    public final static double VISION_THRESHOLD_DISTANCE_M = 1;


    public static final int SIM_CAMERA_RES_WIDTH = 960;
    public static final int SIM_CAMERA_RES_HEIGHT = 720;
    public static final int SIM_CAMERA_FOV_DEG = 90;
    public static final double SIM_CAMERA_AVG_ERROR_PX = 0.35;
    public static final double SIM_CAMERA_ERROR_STD_DEV_PX = 0.1;
    public static final int SIM_CAMERA_FPS = 15;
    public static final double SIM_CAMERA_AVG_LATENCY_MS = 50;
    public static final double SIM_CAMERA_LATENCY_STD_DEV_MS = 15;
    


}

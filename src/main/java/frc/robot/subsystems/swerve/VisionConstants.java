// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.io.Console;
import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {

    //TODO - put real values
    public final static Transform3d robotToCameraTransform3d = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    public static Pose2d robotStartingPose = new Pose2d(0,0, new Rotation2d(0,0));
    public static double useVisionThresholdDistance = 1;
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

    public static final int simCameraResWidth = 960;
    public static final int simCameraResHeight = 720;
    public static final int simCameraFovDeg = 90;
    public static final double simCameraAvgErrorPx = 0.35;
    public static final double simCameraErrorStdDevPx = 0.1;
    public static final int simCameraFps = 15;
    public static final double simCameraAvgLatencyMs = 50;
    public static final double simCameraLatencyStdDevMs = 15;
    


}

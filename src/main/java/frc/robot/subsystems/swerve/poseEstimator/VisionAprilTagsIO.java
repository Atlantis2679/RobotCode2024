// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public final Supplier<Pose3d> poseEstimate = fields.addObject("poseEstimate", this::getRobotPose);
    public final DoubleSupplier cameraTimestampSeconds = fields.addDouble("cameraTimestamp", this::getRobotPoseTimestampSeconds);
    public final BooleanSupplier hasNewRobotPose = fields.addBoolean("hasNewRobotPose", this::getHasNewRobotPose);
    
    protected VisionAprilTagsIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    protected abstract double getRobotPoseTimestampSeconds();

    protected abstract Pose3d getRobotPose();

    protected abstract boolean getHasNewRobotPose();
}

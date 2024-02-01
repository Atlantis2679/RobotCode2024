// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.io;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class VisionIO extends IOBase {

    public final Supplier<Pose3d> photonPoseEstimate = fields.addObject("poseEstimate", this::getPoseEstimate);
    public final DoubleSupplier cameraTimestampSeconds = fields.addDouble("cameraTimestamp", this::getCameraTimestampSeconds);
    protected VisionIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    protected abstract double getCameraTimestampSeconds();

    protected abstract Pose3d getPoseEstimate();

}

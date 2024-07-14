// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectDetection;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.objectDetection.io.ObjectDetectionIO;
import frc.robot.subsystems.objectDetection.io.ObjectDetectionIOCamera;

public class ObjectDetection extends SubsystemBase {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("ObjectDetecion");

  private final ObjectDetectionIO io = new ObjectDetectionIOCamera(fieldsTable);

  public ObjectDetection() {
    fieldsTable.update();
  }

  @Override
  public void periodic() {
  }

  public PhotonPipelineResult getResult() {
    return io.result.get();
  }

  public boolean hasTarget() {
    return io.hasTarget.getAsBoolean();
  }

  public PhotonTrackedTarget getBestResult() {
    return io.bestTarget.get();
  }

}

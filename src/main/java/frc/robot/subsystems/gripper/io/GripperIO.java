// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper.io;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

import java.util.function.BooleanSupplier;

public abstract class GripperIO extends IOBase {
  /** Creates a new GripperIO. */

  public final BooleanSupplier noteDetectorValue = fields.addBoolean("noteDetectorValue", this::getNoteDetectorValue);

  public GripperIO(LogFieldsTable fieldsTable) {
    super(fieldsTable);
  }

  // OUTPUT
  public abstract void setGripperSpeedPrecentOutput(double rollersSpeed);

  // INPUTS
  protected abstract boolean getNoteDetectorValue();
}

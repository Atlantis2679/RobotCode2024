// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.logfields.LogFieldsTable;
import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.RobotMap.Intake.*;

public class GripperIOSparkMax extends GripperIO {

  private final CANSparkMax rollersMotor = new CANSparkMax(ROLLERS_MOTOR_ID,
      MotorType.kBrushless);

  private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);

  /** Creates a new GripperIOSparkMax. */

  public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
  }
  
  @Override
  public void setGripperSpeedPrecentOutput(double speed) {
      rollersMotor.set(speed);
  }
  public boolean getNoteDetectorValue(){
    return !beamBreak.get();
}

}

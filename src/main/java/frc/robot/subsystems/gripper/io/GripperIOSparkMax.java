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

    private final CANSparkMax upperGriperMotor = new CANSparkMax(UPPER_ROLLER_MOTOR_ID,
            MotorType.kBrushless);

    private final CANSparkMax lowerGrippeMotor = new CANSparkMax(LOWER_ROLLER_MOTOR_ID,
            MotorType.kBrushless);

    private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);

    public GripperIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        upperGriperMotor.setSmartCurrentLimit(30);
        lowerGrippeMotor.setSmartCurrentLimit(30);
    }

    @Override
    public void setGripperSpeedPrecentOutput(double upperGripperSpeed, double lowerGripperSpeed) {
        upperGriperMotor.set(upperGripperSpeed);
        lowerGrippeMotor.set(lowerGripperSpeed);
    }

    public boolean getNoteDetectorValue() {
        return !beamBreak.get();
    }

}

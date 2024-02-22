package frc.robot.subsystems.wrist.io;

import frc.lib.logfields.LogFieldsTable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.RobotMap.Intake.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class WristIOSparkMax extends WristIO {
  
    private final CANSparkMax wristMotor = new CANSparkMax(WRIST_MOTOR_ID,
            MotorType.kBrushless);
    private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(WRIST_ENCODER_ID);

    public WristIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        wristMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }



    @Override
    public double getWristAngleDegrees() {
        return (1 - wristEncoder.getAbsolutePosition()) * 360;
    }

}

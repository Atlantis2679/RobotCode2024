package frc.robot.subsystems.intake.io;

import frc.lib.logfields.LogFieldsTable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.RobotMap.Intake.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOSparkMax extends IntakeIO {
    private final CANSparkMax rollersMotor = new CANSparkMax(ROLLERS_MOTOR_ID,
            MotorType.kBrushless);
    private final CANSparkMax wristMotor = new CANSparkMax(WRIST_MOTOR_ID,
            MotorType.kBrushless);
    private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);

    DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);

    public IntakeIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        wristMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    @Override
    public void setRollerSpeedPrecentOutput(double speed) {
        rollersMotor.set(speed);
    }

    @Override
    public double getWristAngleDegrees() {
        return (1 - wristEncoder.getAbsolutePosition()) * 360;
    }

    public boolean getNoteDetectorValue(){
        return !beamBreak.get();
    }
}

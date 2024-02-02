package frc.robot.subsystems.intake.io;

import frc.lib.logfields.LogFieldsTable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOSparkMax extends IntakeIO {
    private final CANSparkMax rollersCANSparkMax = new CANSparkMax(CAN_SPARK_MAX_ROLLERS_ID,
            MotorType.kBrushless);
    private final CANSparkMax wristCANSparkMax = new CANSparkMax(CAN_SPARK_MAX_WRIST_ID,
            MotorType.kBrushless);
    private final DutyCycleEncoder WristEncoder = new DutyCycleEncoder(DUTY_CYCLE_ENCODER_WRIST_ID);

    DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_ID);

    public IntakeIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setWristSpeedPrecentOutput(double wristSpeed) {
        wristCANSparkMax.set(wristSpeed);
    }

    @Override
    public void setRollerSpeedPrecentOutput(double wristSpeed) {
        rollersCANSparkMax.set(wristSpeed);
    }

    @Override
    public double getWristAngleDegrees() {
        return WristEncoder.getAbsolutePosition();
    }

    @Override
    public double getWristSpeed() {
        return wristCANSparkMax.get();
    }


    public boolean getBeamBreakValue(){
        return beamBreak.get();
    }

}

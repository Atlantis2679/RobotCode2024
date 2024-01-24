package frc.robot.subsystems.intake.io;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.intake.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOSparkMax extends IntakeIO {
    private final CANSparkMax intakeCANSparkMax = new CANSparkMax(CAN_SPARK_MAX_INTAKE_ID,
            MotorType.kBrushless);
    private final CANSparkMax intakePJointCANSparkMax = new CANSparkMax(CAN_SPARK_MAX_INTAKE_PIVOT_ID,
            MotorType.kBrushless);
    private final DutyCycleEncoder intakeJointEncoder = new DutyCycleEncoder(DUTY_CYCLE_ENCODER);

    public IntakeIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setJointSpeed(double speedAngleIntake) {
        intakePJointCANSparkMax.set(speedAngleIntake);
    }

    @Override
    public void setIntakeSpeed(double speedIntake) {
        intakeCANSparkMax.set(speedIntake);
    }

    @Override
    public double getJointAngleDegrees() {
        return intakeJointEncoder.getAbsolutePosition();
    }

    @Override
    public double getIntakeSpeed() {
        return intakeCANSparkMax.get();
    }

    @Override
    public double getJointSpeed() {
        return intakePJointCANSparkMax.get();
    }

}

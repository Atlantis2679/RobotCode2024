package frc.robot.subsystems.intake.io;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.intake.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOSparkMax extends IntakeIO {
    private final CANSparkMax intakeCANSparkMax = new CANSparkMax(IntakeConstants.CAN_SPARK_MAX_INTAKE_ID,
            MotorType.kBrushless);
    private final CANSparkMax intakePivotCANSparkMax = new CANSparkMax(IntakeConstants.CAN_SPARK_MAX_INTAKE_PIVOT_ID,
            MotorType.kBrushless);
    private final DutyCycleEncoder intakePivotEncoder = new DutyCycleEncoder(IntakeConstants.DUTY_CYCLE_ENCODER);

    public IntakeIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setAngleIntake(double speedAngleIntake) {
        intakePivotCANSparkMax.set(speedAngleIntake);
    }

    @Override
    public void setIntakeSpeed(double speedIntake) {
        intakeCANSparkMax.set(speedIntake);
    }

    @Override
    public double getJointAngleDegrees() {
        return intakePivotEncoder.getAbsolutePosition();
    }

    @Override
    public double getIntakeSpeed() {
        return intakeCANSparkMax.get();
    }

    @Override
    public double getArmSpeed(){
        return intakePivotCANSparkMax.get();
    }


}

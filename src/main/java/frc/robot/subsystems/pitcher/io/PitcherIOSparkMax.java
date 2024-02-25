package frc.robot.subsystems.pitcher.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.Pitcher.*;

public class PitcherIOSparkMax extends PitcherIO {
    private final CANSparkMax motor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ENCODER_ID);

    public PitcherIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        encoder.setDistancePerRotation(360);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
    }

    @Override
    protected double getAngleDegrees() {
        return -encoder.getDistance();
    }

    @Override
    public void setVoltage(double demandVoltage) {
        motor.setVoltage(demandVoltage);
    }
}

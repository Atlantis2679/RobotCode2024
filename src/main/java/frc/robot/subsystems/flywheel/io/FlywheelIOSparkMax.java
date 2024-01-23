package frc.robot.subsystems.flywheel.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.Flywheel.*;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

public class FlywheelIOSparkMax extends FlywheelIO {
    private final CANSparkMax upperRollerMotor = new CANSparkMax(UPPER_ROLLER_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax lowerRollerMotor = new CANSparkMax(LOWER_ROLLER_MOTOR_ID, MotorType.kBrushless);

    private final Encoder upperRollerEncoder = new Encoder(UPPER_ROLLER_ENCODER_CHANNEL_A, UPPER_ROLLER_ENCODER_CHANNEL_B);
    private final Encoder lowerRollerEncoder = new Encoder(LOWER_ROLLER_ENCODER_CHANNEL_A, LOWER_ROLLER_ENCODER_CHANNEL_B);

    public FlywheelIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        double encodersDistancePerPulse = 1 / AMT103_PULSES_PER_ROUND;
        upperRollerEncoder.setDistancePerPulse(encodersDistancePerPulse);
        lowerRollerEncoder.setDistancePerPulse(encodersDistancePerPulse);
    }

    @Override
    protected double getUpperRollerSpeedRPS() {
        return upperRollerEncoder.getRate();
    }

    @Override
    protected double getLowerRollerSpeedRPS() {
        return lowerRollerEncoder.getRate();
    }

    @Override
    public void setUpperRollerVoltage(double voltageDemand) {
        upperRollerMotor.setVoltage(voltageDemand);
    }

    @Override
    public void setLowerRollerVoltage(double voltageDemand) {
        lowerRollerMotor.setVoltage(voltageDemand);
    }
}

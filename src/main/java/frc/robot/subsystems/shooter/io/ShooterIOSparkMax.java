package frc.robot.subsystems.shooter.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.lib.logfields.LogFieldsTable;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSparkMax extends ShooterIO {
    private final CANSparkMax upperRollerMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax lowerRollerMotor = new CANSparkMax(0, MotorType.kBrushless);

    private final Encoder upperRollerEncoder = new Encoder(0, 0);
    private final Encoder lowerRollerEncoder = new Encoder(0, 0);

    protected ShooterIOSparkMax(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        double distancePerPulse = 1 / AMT103_PULSES_PER_ROUND;
        upperRollerEncoder.setDistancePerPulse(distancePerPulse);
        lowerRollerEncoder.setDistancePerPulse(distancePerPulse);
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
    public void setUpperRollerSpeedVoltage(double voltageDemand) {
        upperRollerMotor.setVoltage(voltageDemand);
    }

    @Override
    public void setLowerRollerSpeedVoltage(double voltageDemand) {
        lowerRollerMotor.setVoltage(voltageDemand);
    }
}

package frc.robot.subsystems.loader.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.lib.logfields.LogFieldsTable;
import static frc.robot.RobotMap.Loader.*;
import static frc.robot.subsystems.loader.LoaderConstants.*;

public class LoaderIOTalon extends LoaderIO {
    private final TalonSRX motor = new TalonSRX(MOTOR_ID);

    public LoaderIOTalon(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        motor.configFactoryDefault();
        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        CURRENT_LIMIT,
                        CURRENT_LIMIT_THRESHOLD,
                        CURRENT_LIMIT_TIME_THRESHOLD));
    }

    @Override
    public void setSpeed(double demandPrecentage) {
        motor.set(ControlMode.PercentOutput, demandPrecentage);
    }
}

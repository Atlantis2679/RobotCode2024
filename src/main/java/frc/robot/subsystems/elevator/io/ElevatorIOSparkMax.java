package frc.robot.subsystems.elevator.io;


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSparkMax extends ElevatorIO {
    private final CANSparkMax motorRight = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax motorLeft = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_LETF_ID, MotorType.kBrushless);
    

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
        motorRight.restoreFactoryDefaults();
        motorLeft.restoreFactoryDefaults();
    }

    @Override
    public void setSpeedRight(double speed){
        motorRight.set(speed);
    }

    @Override
    public void setSpeedLeft(double speed) {
        motorLeft.set(speed);
    }

}


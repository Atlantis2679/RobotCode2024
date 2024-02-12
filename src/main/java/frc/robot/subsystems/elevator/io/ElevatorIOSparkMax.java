package frc.robot.subsystems.elevator.io;


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSparkMax extends ElevatorIO {
    private final CANSparkMax elevatorMotorRight = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax elevatorMotorLeft = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_LETF_ID, MotorType.kBrushless);
    

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
        elevatorMotorRight.follow(elevatorMotorLeft);
    }

    @Override
    public void setSpeedRight(double speed){
        elevatorMotorLeft.set(speed);
    }

}


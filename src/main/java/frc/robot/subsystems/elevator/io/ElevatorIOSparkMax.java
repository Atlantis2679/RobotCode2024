package frc.robot.subsystems.elevator.io;


import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSparkMax extends ElevatorIO {
    private final CANSparkMax elevatorMotorRight = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
    private final CANSparkMax elevatorMotorLeft = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_LETF_ID, MotorType.kBrushless);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(0);
    public final DoubleSupplier hight = fields.addDouble("hight", this::getEncoder);
   
    

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
        elevatorMotorRight.follow(elevatorMotorLeft);
    }

    public double getEncoder(){
        return elevatorEncoder.getAbsolutePosition();
    }

    @Override
    public void setSpeed(double speed){
        elevatorMotorLeft.set(speed);
    }

}


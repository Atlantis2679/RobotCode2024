package frc.robot.subsystems.elevator.io;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSparkMax extends ElevatorIO {
    private final CANSparkMax elevatorMotorRight = new CANSparkMax(0, MotorType.kBrushless);
    private final CANSparkMax elevatorMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(0);
    private final PIDController elevatorPID = new PIDController((ElevatorConstants.KP), (ElevatorConstants.KI), (ElevatorConstants.KD));
    

    public ElevatorIOSparkMax(LogFieldsTable fieldsTable){
        super(fieldsTable);
        elevatorMotorRight.follow(elevatorMotorLeft);
    }

    protected double getEncoder(){
        return elevatorEncoder.getAbsolutePosition();
    }

    public void setSpeed(double speed){
        elevatorMotorLeft.set(speed);
    }

    public void setP(double p) {
        elevatorPID.setP(p);
    }

    public void setI(double i) {
        elevatorPID.setI(i);
    }

    public void setD(double d) {
        elevatorPID.setD(d);
    }

}


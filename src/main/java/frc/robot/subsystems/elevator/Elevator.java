package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;

public class Elevator extends SubsystemBase{
    private final LogFieldsTable fieldsTable = new LogFieldsTable ("Elevator");
    private final ElevatorIO io = new ElevatorIOSparkMax(fieldsTable);

    public void setSpeed(double speed){
        speed = MathUtil.clamp(speed, 0, ElevatorConstants.SPEED_LIMIT);
        io.setSpeed(speed);
    }

    public double getEncoder(){
        return io.getEncoder();
    }

    public void setP (double p){
        io.setP(p);
    }

    public void setI (double i){
        io.setP(i);
    }

    public void setD (double d){
        io.setP(d);
    }

}

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io = new ElevatorIOSparkMax(null);

    public void setSpeed(double speed){
        speed = MathUtil.clamp(speed, 0, 1);
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

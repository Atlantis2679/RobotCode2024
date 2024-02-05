package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorIOSparkMax;

public class Elevator extends SubsystemBase{
    private final LogFieldsTable fieldsTable = new LogFieldsTable ("Elevator");
    private final ElevatorIO io = new ElevatorIOSparkMax(fieldsTable);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
    private final PIDController elevatorPID = new PIDController((ElevatorConstants.KP), (ElevatorConstants.KI), (ElevatorConstants.KD));

    public void setSpeed(double speed){
        speed = MathUtil.clamp(speed, -ElevatorConstants.SPEED_LIMIT, ElevatorConstants.SPEED_LIMIT);
        io.setSpeed(speed);
    }

    public double getEncoder(){
        return io.encoderAngle.getAsDouble();
    }

    public double calculateFeedforward(double desiredAngle, double velocity, boolean usePID) {

        double voltages = feedforward.calculate(Math.toRadians(desiredAngle), velocity);

        if (usePID) {
            voltages += elevatorPID.calculate(getEncoder(), desiredAngle);
        }

        return voltages;
    }

}

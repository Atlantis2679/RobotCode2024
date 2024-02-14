
package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    private final Elevator elevatorRight;
    private final Elevator elevatorLeft;

    public ElevatorCommands(Elevator elevatorLeft, Elevator elevatorRight){
        this.elevatorRight = elevatorRight;
        this.elevatorLeft = elevatorLeft;
    }

    public void manualControl (BooleanSupplier a, BooleanSupplier b, BooleanSupplier c, BooleanSupplier d){
        if (a.getAsBoolean() == true){
            elevatorRight.setSpeedRight(ElevatorConstants.SPEED);
        }
        if (b.getAsBoolean() == true){
            elevatorLeft.setSpeedLeft(ElevatorConstants.SPEED);
        }

        if (c.getAsBoolean() == true){
            elevatorLeft.setSpeedRight(-ElevatorConstants.SPEED);
        }

        if (d.getAsBoolean() == true){
            elevatorLeft.setSpeedLeft(-ElevatorConstants.SPEED);
        }
    }

    public Command mCommand(){
        return manualControl(null, null, null, null).run(() ->);;
    }
}

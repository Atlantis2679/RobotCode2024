
package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    private final Elevator elevatorRight;
    private final Elevator elevatorLeft;
    private final Elevator elevator;

    public ElevatorCommands(Elevator elevatorLeft, Elevator elevatorRight, Elevator elevator){
        this.elevatorRight = elevatorRight;
        this.elevatorLeft = elevatorLeft;
        this.elevator = elevator;
        }

    public Command manualControl (BooleanSupplier isNegative, DoubleSupplier rightButton, DoubleSupplier leftButton){
        return isNegative.getAsBoolean() == false
            ? elevator.run(() -> {
                if (rightButton.getAsDouble() > 0){
                    elevatorRight.setSpeedRight(-ElevatorConstants.SPEED);
                }

                if (leftButton.getAsDouble() > 0){
                    elevatorLeft.setSpeedLeft(ElevatorConstants.SPEED);
                }
            })
            : elevator.run(() -> {
                if (rightButton.getAsDouble() > 0){
                    elevatorRight.setSpeedRight(-ElevatorConstants.SPEED);
                }

                if (leftButton.getAsDouble() > 0){
                    elevatorLeft.setSpeedLeft(ElevatorConstants.SPEED);
                }
            });
    }
}

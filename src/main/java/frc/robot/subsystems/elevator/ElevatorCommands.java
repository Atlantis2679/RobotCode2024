
package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    private final Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command manualControl(DoubleSupplier joystickSupplier, BooleanSupplier isNegative) {
        return elevator.run(() -> elevator.setSpeed(joystickSupplier.getAsDouble(), isNegative.getAsBoolean()))
                .finallyDo(elevator::stop);
    }
}

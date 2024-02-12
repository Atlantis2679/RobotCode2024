
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommands {
    private final Elevator elevator;

    public ElevatorCommands(Elevator elevator){
        this.elevator = elevator;
    }
    public Command openRight(Double speed){
        return elevator.setSpeedRight(speed);
    }
}

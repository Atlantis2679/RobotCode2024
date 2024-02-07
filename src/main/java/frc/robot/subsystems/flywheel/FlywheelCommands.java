package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.commands.RotateFlywheel;

public class FlywheelCommands {
    private final Flywheel flywheel;

    public FlywheelCommands(Flywheel flywheel) {
        this.flywheel = flywheel;
    }

    public Command rotate(DoubleSupplier upperRollerSpeedSupplier, DoubleSupplier lowerRollerSpeedSupplier) {
        return new RotateFlywheel(flywheel, upperRollerSpeedSupplier, lowerRollerSpeedSupplier);
    }
}

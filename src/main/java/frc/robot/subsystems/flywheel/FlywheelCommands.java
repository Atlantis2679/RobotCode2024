package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelCommands {
    private final Flywheel flywheel;

    public FlywheelCommands(Flywheel flywheel) {
        this.flywheel = flywheel;
    }

    public Command spin(DoubleSupplier upperRollerSpeedSupplierRPS, DoubleSupplier lowerRollerSpeedSupplierRPS) {
        return flywheel
                .runOnce(() -> flywheel.resetPIDs())
                .andThen(flywheel.run(() -> {
                    double upperRollerVoltage = flywheel
                            .calculateUpperRollerVoltageForSpeed(upperRollerSpeedSupplierRPS.getAsDouble());
                    double lowerRollerVoltage = flywheel
                            .calculateLowerRollerVoltageForSpeed(lowerRollerSpeedSupplierRPS.getAsDouble());
                    flywheel.setSpeed(upperRollerVoltage, lowerRollerVoltage);
                }))
                .finallyDo(flywheel::stop);
    }

    public Command manualController(DoubleSupplier upperRollerSpeedJoystick, DoubleSupplier lowerRollerSpeedJoystick) {
        return flywheel.run(() -> 
            flywheel.setSpeed(upperRollerSpeedJoystick.getAsDouble(), lowerRollerSpeedJoystick.getAsDouble())
        );
    }
}

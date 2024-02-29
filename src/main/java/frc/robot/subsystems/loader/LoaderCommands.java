package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class LoaderCommands {
    private final Loader loader;

    public LoaderCommands(Loader loader) {
        this.loader = loader;
    }

    public Command spin(double spinSpeedPercentOutput) {
        return loader.startEnd(() -> loader.setSpeed(spinSpeedPercentOutput), loader::stop);
    }

    public Command manualController(DoubleSupplier joystick) {
        return loader.run(() ->
                loader.setSpeed(joystick.getAsDouble())
        );
    }
}

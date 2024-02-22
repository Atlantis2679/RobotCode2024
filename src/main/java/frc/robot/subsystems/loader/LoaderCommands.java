package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoaderCommands {
    private final Loader loader;

    public LoaderCommands(Loader loader) {
        this.loader = loader;
    }

    public Command release(double spinSpeedPercentOutput) {
        return loader.runOnce(() -> loader.setSpeed(spinSpeedPercentOutput))
                .andThen(Commands.waitSeconds(0.6))
                .finallyDo(() -> loader.setSpeed(0));
    }

    public Command manualController(DoubleSupplier joystick) {
        return loader.run(() ->
                loader.setSpeed(joystick.getAsDouble())
        );
    }
}

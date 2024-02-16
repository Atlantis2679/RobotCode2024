package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoaderCommands {
    private final Loader loader;

    public LoaderCommands(Loader loader) {
        this.loader = loader;
    }

    public Command release() {
        return loader.runOnce(() -> loader.setSpeed(0))
                .andThen(Commands.waitSeconds(0.6))
                .finallyDo(() -> loader.setSpeed(0));
    }
}

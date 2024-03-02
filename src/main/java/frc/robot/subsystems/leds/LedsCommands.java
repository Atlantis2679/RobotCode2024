package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj2.command.Command;

public class LedsCommands {
    private final Leds leds;

    public LedsCommands(Leds leds) {
        this.leds = leds;
    }

    public Command set00BEBE() {
        return leds.run(() -> leds.setColor(0, 190, 190));
    }

    public Command setRed() {
        return leds.run(() -> leds.setColor(255, 0, 0));

    }
        public Command setGreen() {
        return leds.run(() -> leds.setColor(0, 255, 0));

    }
}

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

    public Command setPurple() {
        return leds.run(() -> leds.setColor(138, 43, 226));
    }

    public Command setPink() {
        return leds.run(() -> leds.setColor(255, 0, 255));
    }

    public Command setOrange() {
        return leds.run(() -> leds.setColor(255, 165, 0));
    }

//     public Command setRainbowColor(){
//         return leds.run(() -> { 
//             set00BEBE().until().andThen(setGreen()).wait(2000).andThen(setOrange()).wait(2000).andThen(setPink()).wait(2000).andThen(setPurple()).wait(2000).andThen(setRed());
// })
    // }

}

package frc.robot.subsystems.pitcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.valueholders.ValueHolder;

public class PitcherCommands {
    private final Pitcher pitcher;

    public PitcherCommands(Pitcher pitcher) {
        this.pitcher = pitcher;
    }

    public Command adjustToAngle(double goalAngleDegree) {
        Timer timer = new Timer();
        ValueHolder<TrapezoidProfile.State> initialState = new ValueHolder<>(null);
        TrapezoidProfile.State goalState = new TrapezoidProfile.State(goalAngleDegree, 0);

        return pitcher.runOnce(() -> {
            timer.restart();
            initialState.set(new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()));
        }).andThen(Commands.run(() -> {
            TrapezoidProfile.State state = pitcher.calculateTrapezoidProfile(timer.get(), initialState.get(),
                    goalState);
            pitcher.setVoltage(pitcher.calculateFeedforward(state.position, state.velocity, true));
        }));
    }

    public Command adjustToAngle(DoubleSupplier goalAngleDegrees) {
        ValueHolder<TrapezoidProfile.State> desiredState = new ValueHolder<TrapezoidProfile.State>(null);

        return pitcher.runOnce(() -> {
            desiredState.set(new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()));
        }).andThen(Commands.run(() -> {
            desiredState.set(pitcher.calculateTrapezoidProfile(
                    0.02,
                    desiredState.get(),
                    new TrapezoidProfile.State(goalAngleDegrees.getAsDouble(), 0)));

            pitcher.setVoltage(pitcher.calculateFeedforward(desiredState.get().position, desiredState.get().velocity, true));
        }));
    }

    public Command controller(DoubleSupplier joystick) {
        return pitcher.run(() -> {
            double feedforwardResult = pitcher.calculateFeedforward(pitcher.getAngleDegrees(), 0, false);
            pitcher.setVoltage(feedforwardResult + joystick.getAsDouble() * 5);
        });
    }
}

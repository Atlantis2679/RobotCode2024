package frc.robot.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneableCommand;

public class TuneableTrapezoidProfileCommand extends TuneableCommand {
    private final TuneableTrapezoidProfile profile;
    private final Consumer<State> outputConsumer;
    private final Supplier<State> goalSupplier;
    private final Supplier<State> currentStateSupplier;
    private final boolean shouldFinish;
    private final Timer timer = new Timer();

    public TuneableTrapezoidProfileCommand(
            TuneableTrapezoidProfile profile,
            Consumer<State> outputConsumer,
            Supplier<State> goalSupplier,
            Supplier<State> currentStateSupplier,
            boolean shouldFinish,
            Subsystem... requirements) {
        this.profile = profile;
        this.outputConsumer = outputConsumer;
        this.goalSupplier = goalSupplier;
        this.currentStateSupplier = currentStateSupplier;
        this.shouldFinish = shouldFinish;
        addRequirements(requirements);
    }

    public TuneableTrapezoidProfileCommand(
            TuneableTrapezoidProfile profile,
            Consumer<State> outputConsumer,
            Supplier<State> goalSupplier,
            Supplier<State> currentStateSupplier,
            Subsystem... requirements) {
        this(profile, outputConsumer, goalSupplier, currentStateSupplier, true, requirements);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        outputConsumer.accept(profile.calculate(timer.get(), currentStateSupplier.get(), goalSupplier.get()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return shouldFinish && timer.hasElapsed(profile.totalTime());
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        profile.initTuneable(builder);
    }
}

package frc.robot.subsystems.pitcher.command;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pitcher.Pitcher;

import java.util.function.DoubleSupplier;

public class AdjustPitcherToMovingAngle extends Command {
    private final Pitcher pitcher;
    private final DoubleSupplier goalAngleDegrees;

    public AdjustPitcherToMovingAngle(Pitcher pitcher, DoubleSupplier goalAngleDegrees) {
        this.pitcher = pitcher;
        this.goalAngleDegrees = goalAngleDegrees;
        addRequirements(pitcher);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = pitcher.calculateTrapezoidProfile(
                0.02,
                new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()),
                new TrapezoidProfile.State(goalAngleDegrees.getAsDouble(), 0));

        pitcher.setVoltage(pitcher.calculateFeedforward(state.position, state.velocity, true));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.subsystems.pitcher.command;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pitcher.Pitcher;

public class AdjustPitcherToStaticAngle extends Command {
    private final Pitcher pitcher;
    private final Timer timer = new Timer();
    private TrapezoidProfile.State initialState;
    private TrapezoidProfile.State goalState;

    public AdjustPitcherToStaticAngle(Pitcher pitcher, double goalAngleDegrees) {
        this.pitcher = pitcher;
        this.goalState = new TrapezoidProfile.State(goalAngleDegrees, 0);
        addRequirements(pitcher);
    }

    @Override
    public void initialize() {
        timer.restart();
        initialState = new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec());
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = pitcher.calculateTrapezoidProfile(timer.get(), initialState, goalState);
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

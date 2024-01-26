package frc.robot.subsystems.pitcher;

import frc.lib.tuneables.TuneableCommand;
import frc.robot.subsystems.pitcher.command.AdjustPitcherToAngle;

public class PitcherCommands {
    private final Pitcher pitcher;

    public PitcherCommands(Pitcher pitcher) {
        this.pitcher = pitcher;
    }

    public TuneableCommand adjustToAngle(double goalAngleDegree) {
        return new AdjustPitcherToAngle(pitcher, goalAngleDegree);
    }
}

package frc.robot.subsystems.pitcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.lib.tuneables.TuneableCommand;

import static frc.robot.subsystems.pitcher.PitcherConstants.RotateToAngle.*;

public class PitcherCommands {
    private final Pitcher pitcher;

    public PitcherCommands(Pitcher pitcher) {
        this.pitcher = pitcher;
    }

    public Command rotateToAngle(DoubleSupplier goalAngleDegreeSupplier) {
        PIDController pidController = new PIDController(KP, KI, KD);
        TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0, 0));
        return TuneableCommand.wrap(
                new TrapezoidProfileCommand(
                    trapezoidProfile,
                    (state) -> {},
                    () -> new TrapezoidProfile.State(goalAngleDegreeSupplier.getAsDouble(), 0),
                    () -> new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()),
                    pitcher
                ),
                (builder) -> {
                    builder.addChild("PID", pidController);
                });
    }
}

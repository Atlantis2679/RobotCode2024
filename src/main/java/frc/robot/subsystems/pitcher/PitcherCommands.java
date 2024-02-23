package frc.robot.subsystems.pitcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.valueholders.ValueHolder;
import frc.robot.subsystems.pitcher.PitcherConstants.AdjustToAngle;

public class PitcherCommands {
    private final Pitcher pitcher;

    public PitcherCommands(Pitcher pitcher) {
        this.pitcher = pitcher;
    }

    public Command adjustToAngle(double goalAngleDegrees) {
        return adjustToAngle(() -> goalAngleDegrees);
    }

    public Command adjustToAngle(DoubleSupplier goalAngleDegrees) {
        ValueHolder<TrapezoidProfile.State> refrenceState = new ValueHolder<TrapezoidProfile.State>(null);

        return pitcher.runOnce(() -> {
            refrenceState.set(new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()));
        }).andThen(Commands.run(() -> {
            refrenceState.set(pitcher.calculateTrapezoidProfile(
                    0.02,
                    AdjustToAngle.USE_CLOSED_LOOP_PROFILE
                            ? new TrapezoidProfile.State(pitcher.getAngleDegrees(), refrenceState.get().velocity)
                            : refrenceState.get(),
                    new TrapezoidProfile.State(goalAngleDegrees.getAsDouble(), 0)));

            pitcher.setVoltage(
                    pitcher.calculateFeedforward(refrenceState.get().position, refrenceState.get().velocity, true));
        })).finallyDo(() -> pitcher.setVoltage(0));
    }

    public Command manualController(DoubleSupplier joystick) {
        return pitcher.run(() -> {
            double feedforwardResult = pitcher.calculateFeedforward(pitcher.getAngleDegrees(), 0, false);
            pitcher.setVoltage(feedforwardResult + joystick.getAsDouble() * 4);
        });
    }
}

package frc.robot.subsystems.wrist;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.valueholders.ValueHolder;
import frc.robot.subsystems.wrist.WristConstants.ManualController;
import frc.robot.subsystems.wrist.WristConstants.MoveToAngle;


public class WristCommands {
    private final Wrist wrist;

    public WristCommands(Wrist intake) {
        this.wrist = intake;
    }

    public Command moveToAngle(double goalAngleDegrees) {
        ValueHolder<TrapezoidProfile.State> refrenceState = new ValueHolder<TrapezoidProfile.State>(null);

        return wrist.runOnce(() -> {
            wrist.resetPID();
            refrenceState.set(new TrapezoidProfile.State(wrist.getAbsoluteAngleDegrees(), 0));
        }).andThen(Commands.run(() -> {
            refrenceState.set(wrist.calculateTrapezoidProfile(
                    0.02,
                    MoveToAngle.USE_CLOSED_LOOP_PROFILE
                            ? new TrapezoidProfile.State(wrist.getAbsoluteAngleDegrees(), refrenceState.get().velocity)
                            : refrenceState.get(),
                    new TrapezoidProfile.State(goalAngleDegrees, 0)));

            wrist.setWristVoltage(
                    wrist.calculateFeedforward(refrenceState.get().position, refrenceState.get().velocity, true));
        }));
    }

    public Command manualController(DoubleSupplier wristSpeed) {
        return wrist.run(() -> {
            double feedforwardResult = wrist.calculateFeedforward(
                    wrist.getAbsoluteAngleDegrees(),
                    0,
                    false);
            wrist.setWristVoltage(feedforwardResult + wristSpeed.getAsDouble() * ManualController.SPEED_MULTIPLIER);
        });
    }
 
}

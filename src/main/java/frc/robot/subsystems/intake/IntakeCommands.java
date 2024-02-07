package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.valueholders.ValueHolder;

public class IntakeCommands {
    private final Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command moveToAngle(double goalAngleDegrees) {
        ValueHolder<TrapezoidProfile.State> refrenceState = new ValueHolder<TrapezoidProfile.State>(null);

        return intake.runOnce(() -> {
            refrenceState.set(new TrapezoidProfile.State(intake.getAbsoluteAngleDegrees(), 0));
        }).andThen(Commands.run(() -> {
            refrenceState.set(intake.calculateTrapezoidProfile(
                    0.02,
                    USE_CLOSED_LOOP_PROFILE
                            ? new TrapezoidProfile.State(intake.getAbsoluteAngleDegrees(), refrenceState.get().velocity)
                            : refrenceState.get(),
                    new TrapezoidProfile.State(goalAngleDegrees, 0)));

            intake.setWristVoltage(
                    intake.calculateFeedforward(refrenceState.get().position, refrenceState.get().velocity, true));
        }));
    }

    public Command open() {
        return moveToAngle(COLLECTING_WRIST_ANGLE_DEGREE)
                .andThen(() -> intake.setSpeedRollers(COLLECTING_ROLLERS_SPEED));
    }

    public Command close() {
        return moveToAngle(CLOSED_WRIST_ANGLE_DEGREE);
    }

}

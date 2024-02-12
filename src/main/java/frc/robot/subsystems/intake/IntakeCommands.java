package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.valueholders.ValueHolder;
import frc.robot.subsystems.intake.IntakeConstants.Close;
import frc.robot.subsystems.intake.IntakeConstants.ManualController;
import frc.robot.subsystems.intake.IntakeConstants.MoveToAngle;
import frc.robot.subsystems.intake.IntakeConstants.Open;
import frc.robot.subsystems.intake.IntakeConstants.aimToAmp;

public class IntakeCommands {
    private final Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    private Command moveToAngle(double goalAngleDegrees) {
        ValueHolder<TrapezoidProfile.State> refrenceState = new ValueHolder<TrapezoidProfile.State>(null);

        return intake.runOnce(() -> {
            intake.resetPID();
            refrenceState.set(new TrapezoidProfile.State(intake.getAbsoluteAngleDegrees(), 0));
        }).andThen(Commands.run(() -> {
            refrenceState.set(intake.calculateTrapezoidProfile(
                    0.02,
                    MoveToAngle.USE_CLOSED_LOOP_PROFILE
                            ? new TrapezoidProfile.State(intake.getAbsoluteAngleDegrees(), refrenceState.get().velocity)
                            : refrenceState.get(),
                    new TrapezoidProfile.State(goalAngleDegrees, 0)));

            intake.setWristVoltage(
                    intake.calculateFeedforward(refrenceState.get().position, refrenceState.get().velocity, true));
        }));
    }

    public Command open() {
        return moveToAngle(Open.COLLECTING_WRIST_ANGLE_DEGREE)
                .andThen(() -> intake.setSpeedRollers(COLLECTING_ROLLERS_SPEED));
    }

    public Command close() {
        return moveToAngle(Close.CLOSED_WRIST_ANGLE_DEGREE);
    }

    public Command aimToAmp(){
        return moveToAngle(aimToAmp.AIM_TO_AMP_WRIST_DEGREE);
    }

    public Command manualController(DoubleSupplier speed) {
        return intake.run(() -> {
            double feedforwardResult = intake.calculateFeedforward(
                    intake.getAbsoluteAngleDegrees(),
                    0,
                    false);

            intake.setWristVoltage(feedforwardResult + speed.getAsDouble() * ManualController.SPEED_MULTIPLIER);
        });
    }
    
}

package frc.robot.subsystems.pitcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneableCommand;
import frc.robot.utils.TuneableArmFeedforward;
import frc.robot.utils.TuneableTrapezoidProfile;
import frc.robot.utils.TuneableTrapezoidProfileCommand;

import static frc.robot.subsystems.pitcher.PitcherConstants.RotateToAngle.*;

public class PitcherCommands {
    private final Pitcher pitcher;

    public PitcherCommands(Pitcher pitcher) {
        this.pitcher = pitcher;
    }

    public TuneableCommand adjustToAngle(DoubleSupplier goalAngleDegreeSupplier) {
        PIDController pidController = new PIDController(KP, KI, KD);
        TuneableArmFeedforward armFeedforward = new TuneableArmFeedforward(KS, KG, KV);
        LogFieldsTable fieldsTable = pitcher.getFieldsSubTable("Adjust Angle CMD");

        return new TuneableTrapezoidProfileCommand(
                new TuneableTrapezoidProfile(
                        new TrapezoidProfile.Constraints(10, 10)),
                (state) -> {
                    double feedforwardResult = armFeedforward.calculate(Math.toRadians(state.position), state.velocity);
                    double pidResult = pidController.calculate(pitcher.getAngleDegrees(), state.position);
                    pitcher.setVoltage(pidResult + feedforwardResult);
                    fieldsTable.recordOutput("desired position", state.position);
                    fieldsTable.recordOutput("desired velocity", state.velocity);
                    fieldsTable.recordOutput("feedforward result", feedforwardResult);
                    fieldsTable.recordOutput("pid result", pidResult);
                },
                () -> new TrapezoidProfile.State(goalAngleDegreeSupplier.getAsDouble(), 0),
                () -> new TrapezoidProfile.State(pitcher.getAngleDegrees(), pitcher.getVelocityDegPerSec()),
                false,
                pitcher)
                .extendTuneable((builder) -> {
                    builder.setSendableType(SendableType.LIST);
                    builder.addChild("PID", pidController);
                    armFeedforward.initTuneable(builder);
                });
    }
}

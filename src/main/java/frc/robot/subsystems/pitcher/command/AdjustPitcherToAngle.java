package frc.robot.subsystems.pitcher.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneableCommand;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.utils.TuneableArmFeedforward;
import frc.robot.utils.TuneableTrapezoidProfile;

import static frc.robot.subsystems.pitcher.PitcherConstants.AdjustToAngle.*;

public class AdjustPitcherToAngle extends TuneableCommand {
    private final Pitcher pitcher;
    private final PIDController pidController = new PIDController(KP, KI, KD);
    private final TuneableArmFeedforward feedforward = new TuneableArmFeedforward(KS, KG, KV);
    private final TuneableTrapezoidProfile profile = new TuneableTrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC));
    private final LogFieldsTable fieldsTable;
    private final Timer timer = new Timer();
    private TrapezoidProfile.State initialState;
    private TrapezoidProfile.State goalState;

    public AdjustPitcherToAngle(Pitcher pitcher, double goalAngleDegrees) {
        this.pitcher = pitcher;
        fieldsTable = pitcher.getSubFieldsTable("Adjust Angle CMD");
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
        TrapezoidProfile.State state = profile.calculate(timer.get(), initialState, goalState);
        double feedforwardResult = feedforward.calculate(Math.toRadians(state.position), state.velocity);
        double pidResult = pidController.calculate(pitcher.getAngleDegrees(), state.position);
        pitcher.setVoltage(pidResult + feedforwardResult);
        fieldsTable.recordOutput("desired position", state.position);
        fieldsTable.recordOutput("desired velocity", state.velocity);
        fieldsTable.recordOutput("feedforward result", feedforwardResult);
        fieldsTable.recordOutput("pid result", pidResult);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        profile.initTuneable(builder);
        feedforward.initTuneable(builder);
        builder.addChild("PID", pidController);
        builder.addDoubleProperty("goal angle degrees", () -> goalState.position, (newPosition) -> {
            goalState = new TrapezoidProfile.State(newPosition, 0);
            end(true);
            initialize();
        });
    }
}

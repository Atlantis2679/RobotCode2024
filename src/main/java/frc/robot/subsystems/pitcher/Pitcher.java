package frc.robot.subsystems.pitcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.tuneables.extensions.TuneableArmFeedforward;
import frc.lib.tuneables.extensions.TuneableTrapezoidProfile;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.subsystems.pitcher.io.PitcherIO;
import frc.robot.subsystems.pitcher.io.PitcherIOSim;
import frc.robot.subsystems.pitcher.io.PitcherIOSparkMax;

import static frc.robot.subsystems.pitcher.PitcherConstants.*;

public class Pitcher extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final PitcherIO io = Robot.isSimulation()
            ? new PitcherIOSim(fieldsTable)
            : new PitcherIOSparkMax(fieldsTable);
    private final TuneablesTable tuneablesTable = new TuneablesTable(SendableType.LIST);
    private final DoubleHolder angleOffsetDegrees = tuneablesTable.addNumber("Angle Offset Degrees",
            ANGLE_OFFSET_DEGREES);

    private final PIDController pidController = new PIDController(KP, KI, KD);
    private final TuneableArmFeedforward feedforward = new TuneableArmFeedforward(KS, KG, KV);
    private final TuneableTrapezoidProfile trapezoidProfile = new TuneableTrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VELOCITY_DEG_PER_SEC, MAX_ACCELERATION_DEG_PER_SEC));

    private final PitcherVisualizer realStateVisualizer = new PitcherVisualizer(fieldsTable, "Real Mechanism",
            new Color8Bit("#00BEBE"));

    private final PitcherVisualizer desiredStateVisualizer = new PitcherVisualizer(fieldsTable, "Desired Mechanism",
            new Color8Bit(255, 255, 255));

    private double lastAngleDegree;
    private double velocityDegPerSec;

    public Pitcher() {
        fieldsTable.update();
        lastAngleDegree = getAngleDegrees();

        TuneablesManager.add(getName(), (Tuneable) this);
    }

    @Override
    public void periodic() {
        velocityDegPerSec = (getAngleDegrees() - lastAngleDegree) / 0.02;
        lastAngleDegree = getAngleDegrees();

        realStateVisualizer.update(getAngleDegrees());

        fieldsTable.recordOutput("Velocity RadPerSec", getVelocityDegPerSec());
        fieldsTable.recordOutput("Angle Degrees", getAngleDegrees());
    }

    public LogFieldsTable getSubFieldsTable(String name) {
        return fieldsTable.getSubTable(name);
    }

    public void setVoltage(double voltageDemand) {
        fieldsTable.recordOutput("Requested Voltage", voltageDemand);
        voltageDemand = MathUtil.clamp(voltageDemand, -MAX_VOLTAGE, MAX_VOLTAGE);
        fieldsTable.recordOutput("Actual Voltage", voltageDemand);
        io.setVoltage(voltageDemand);
    }

    public double getAngleDegrees() {
        return io.angleDegrees.getAsDouble() - angleOffsetDegrees.get();
    }

    public double getVelocityDegPerSec() {
        return velocityDegPerSec;
    }

    public double calculateFeedforward(double angleDegrees, double velocityDegPerSec, boolean usePID) {
        double feedforwardResult = feedforward.calculate(
                Math.toRadians(angleDegrees),
                velocityDegPerSec);

        double pidResult = usePID
                ? pidController.calculate(getAngleDegrees(), angleDegrees)
                : 0;

        fieldsTable.recordOutput("Feedforward Result", feedforwardResult);
        fieldsTable.recordOutput("PID Result", pidResult);

        return feedforwardResult + pidResult;
    }

    public TrapezoidProfile.State calculateTrapezoidProfile(
            double time,
            TrapezoidProfile.State initalState,
            TrapezoidProfile.State goalState) {
        TrapezoidProfile.State state = trapezoidProfile.calculate(time, initalState, goalState);
        fieldsTable.recordOutput("Desired State Position", state.position);
        fieldsTable.recordOutput("Desired State Velocity", state.velocity);
        fieldsTable.recordOutput("Goal Position", goalState.position);
        desiredStateVisualizer.update(state.position);
        return state;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        tuneablesTable.initTuneable(builder);
        builder.addChild("Reset Angle", new InstantCommand(() -> {
            angleOffsetDegrees.set(io.angleDegrees.getAsDouble());
        }));
        builder.addChild("PID", pidController);
        feedforward.initTuneable(builder);
        trapezoidProfile.initTuneable(builder);
    }
}

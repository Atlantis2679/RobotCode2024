package frc.robot.subsystems.pitcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.subsystems.pitcher.io.PitcherIO;
import frc.robot.subsystems.pitcher.io.PitcherIOSparkMax;

import static frc.robot.subsystems.pitcher.PitcherConstants.*;

public class Pitcher extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final PitcherIO io = new PitcherIOSparkMax(fieldsTable);
    private final TuneablesTable tuneablesTable = new TuneablesTable(SendableType.LIST);
    private final DoubleHolder angleOffsetDegrees = tuneablesTable.addNumber("Angle Offset Degrees", ANGLE_OFFSET_DEGREES);

    private double lastAngleDegree = 0;
    private double velocityDegPerSec = 0;

    public Pitcher() {
        fieldsTable.update();
        periodic();
    }

    @Override
    public void periodic() {
        velocityDegPerSec = (lastAngleDegree - getAngleDegrees()) * 0.02;
        lastAngleDegree = getAngleDegrees();
    }

    public void setVoltage(double voltageDemand) {
        io.setVoltage(MathUtil.clamp(voltageDemand, -MAX_VOLTAGE, MAX_VOLTAGE));
    }

    public double getAngleDegrees() {
        return io.angleDegrees.getAsDouble() - angleOffsetDegrees.get();
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        tuneablesTable.initTuneable(builder);
        builder.addChild("Reset Angle", new InstantCommand(() -> {
            angleOffsetDegrees.set(io.angleDegrees.getAsDouble());
        }));
    }

    public double getVelocityDegPerSec() {
        return velocityDegPerSec;
    }
}

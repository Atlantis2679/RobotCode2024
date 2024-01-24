package frc.robot.subsystems.pitcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesTable;
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

    private double lastAngleDegree;
    private double velocityDegPerSec;
    private double lastTimestampSec = Timer.getFPGATimestamp();

    public Pitcher() {
        fieldsTable.update();
        lastAngleDegree = getAngleDegrees();
    }

    @Override
    public void periodic() {
        double currTimestampSec = Timer.getFPGATimestamp();
        double cycleTimeSec = lastTimestampSec - currTimestampSec;
        velocityDegPerSec = (getAngleDegrees() - lastAngleDegree) / cycleTimeSec;
        lastAngleDegree = getAngleDegrees();
        lastTimestampSec = currTimestampSec;

        fieldsTable.recordOutput("Velocity RadPerSec", getVelocityDegPerSec());
        fieldsTable.recordOutput("Angle Degrees", getAngleDegrees());
    }

    public LogFieldsTable getFieldsSubTable(String name) {
        return fieldsTable.getSubTable(name);
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

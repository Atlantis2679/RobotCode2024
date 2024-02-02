package frc.robot.subsystems.pitcher.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class PitcherIO extends IOBase {
    public final DoubleSupplier angleDegrees = fields.addDouble("angleDegree", this::getAngleDegrees);

    protected PitcherIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getAngleDegrees();

    // Outputs

    public abstract void setVoltage(double demandVoltage);
}

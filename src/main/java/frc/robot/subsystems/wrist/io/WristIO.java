package frc.robot.subsystems.wrist.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class WristIO extends IOBase {
    public final DoubleSupplier wristAngleDegrees = fields.addDouble("wristAngleDegrees", this::getWristAngleDegrees);

    public WristIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // INPUTS
    protected abstract double getWristAngleDegrees();

    // OUTPUTS

    public abstract void setWristVoltage(double wristSpeed);
}
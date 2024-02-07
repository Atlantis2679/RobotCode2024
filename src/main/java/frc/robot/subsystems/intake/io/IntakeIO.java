package frc.robot.subsystems.intake.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {
    public final DoubleSupplier wristAngleDegrees = fields.addDouble("wristAngleDegrees", this::getWristAngleDegrees);
    public final BooleanSupplier noteDetectorValue = fields.addBoolean("noteDetectorValue", this::getNoteDetectorValue);

    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // INPUTS
    protected abstract double getWristAngleDegrees();

    protected abstract boolean getNoteDetectorValue();

    // OUTPUTS
    public abstract void setRollerSpeedPrecentOutput(double rollersSpeed);

    public abstract void setWristVoltage(double wristSpeed);
}
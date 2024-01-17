package frc.robot.subsystems.intake.io;

import java.util.function.BooleanSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {
    public final BooleanSupplier jointAngleDegrees = fields.addBoolean("jointAngleDegrees", this::getJointAngleDegrees);

    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs
    protected abstract boolean getJointAngleDegrees();

    // outputs
    public abstract void setIntakeSpeed();
}
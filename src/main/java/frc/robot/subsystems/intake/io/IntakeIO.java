package frc.robot.subsystems.intake.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {

    public final DoubleSupplier wristAngleDegrees = fields.addDouble("jointAngleDegrees", this::getWristAngleDegrees);
    public final DoubleSupplier rollersSpeed = fields.addDouble("intakeSpeed", this::getRollersSpeed);
    public final DoubleSupplier wristSpeed = fields.addDouble("jointSpeed", this::getWristSpeed);

    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getWristAngleDegrees();

    protected abstract double getRollersSpeed();

    protected abstract double getWristSpeed();

    public abstract void setRollerSpeed(double rollersSpeed);

    public abstract void setWristSpeed(double wristSpeed);

}
package frc.robot.subsystems.intake.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {

    public final DoubleSupplier jointAngleDegrees = fields.addDouble("jointAngleDegrees", this::getJointAngleDegrees);
    public final DoubleSupplier CurrentIntakeSpeed = fields.addDouble("CurrentIntakeSpeed", this::getCurrentIntakeSpeed);

    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    

    // inputs
    protected abstract double getJointAngleDegrees();

    protected abstract double getCurrentIntakeSpeed();

    // outputs
    public abstract void setIntakeSpeed(double speedIntake);

    public abstract void setAngleIntake(double speedAngleIntake);

}
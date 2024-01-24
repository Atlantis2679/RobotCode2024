package frc.robot.subsystems.intake.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {

    public final DoubleSupplier jointAngleDegrees = fields.addDouble("jointAngleDegrees", this::getJointAngleDegrees);
    public final DoubleSupplier intakeSpeed = fields.addDouble("intakeSpeed", this::getIntakeSpeed);
    public final DoubleSupplier jointSpeed = fields.addDouble("jointSpeed", this::getJointSpeed);

    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getJointAngleDegrees();

    protected abstract double getIntakeSpeed();

    protected abstract double getJointSpeed();

    public abstract void setIntakeSpeed(double speedIntake);

    public abstract void setJointSpeed(double speedAngleIntake);

}
package frc.robot.subsystems.intake.io;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class IntakeIO extends IOBase {

    public final DoubleSupplier wristAngleDegrees = fields.addDouble("jointAngleDegrees", this::getWristAngleDegrees);
    public final DoubleSupplier wristSpeed = fields.addDouble("jointSpeed", this::getWristSpeed);
    public final BooleanSupplier beamBreakValue = fields.addBoolean("beamBreakValue", this::getBeamBreakValue);
    public IntakeIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double getWristAngleDegrees();

    protected abstract double getWristSpeed();

    protected abstract boolean getBeamBreakValue();


    public abstract void setRollerSpeedPrecentOutput(double rollersSpeed);

    public abstract void setWristSpeedPrecentOutput(double wristSpeed);



    

}
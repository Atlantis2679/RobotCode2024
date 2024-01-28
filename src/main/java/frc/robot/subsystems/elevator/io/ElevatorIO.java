package frc.robot.subsystems.elevator.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase{
public final DoubleSupplier encoderValue = fields.addDouble("encoderValue", this::getEncoder);

    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    protected abstract double getEncoder();

    public abstract void setSpeed(double demand);
}

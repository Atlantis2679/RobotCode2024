package frc.robot.subsystems.elevator.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase{
public final DoubleSupplier getEncoder = fields.addDouble("getEncoder", this::getEncoder);

    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }
    protected abstract double getEncoder();

    public abstract void setSpeed(double demand);
}

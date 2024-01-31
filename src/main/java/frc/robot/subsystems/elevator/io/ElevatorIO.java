package frc.robot.subsystems.elevator.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase{

    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract double getEncoder();

    public abstract void setSpeed(double demand);

    public abstract void setP (double demand);

    public abstract void setI (double demand);

    public abstract void setD (double demand);
}

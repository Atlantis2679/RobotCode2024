package frc.robot.subsystems.elevator.io;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase{

    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract void setSpeedRight(double speed);

    public abstract void setSpeedLeft(double speed);
}

package frc.robot.subsystems.elevator.io;

import java.util.function.BooleanSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ElevatorIO extends IOBase {

    public final BooleanSupplier isElevatorLeftDown = fields.addBoolean("is the elevator left down",
            this::getLeftLimitSwitchValue);
    public final BooleanSupplier isElevatorRightDown = fields.addBoolean("is the elevator right down",
            this::getRightLimitSwitchValue);

    public ElevatorIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract void setSpeedRight(double speed);

    public abstract void setSpeedLeft(double speed);

    protected abstract boolean getLeftLimitSwitchValue();

    protected abstract boolean getRightLimitSwitchValue();

}

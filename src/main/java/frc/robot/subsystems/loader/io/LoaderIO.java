package frc.robot.subsystems.loader.io;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class LoaderIO extends IOBase {
    protected LoaderIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // OUTPUTS

    public abstract void setSpeed(double demandPrecentage);    
}

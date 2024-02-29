package frc.robot.subsystems.loader.io;

import java.util.function.BooleanSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class LoaderIO extends IOBase {
    public final BooleanSupplier noteDetectorvalue = fields.addBoolean("noteDetectorValue", this::getNoteDetectorValue);

    protected LoaderIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
        
    }

    // OUTPUTS

    public abstract void setSpeed(double demandPrecentage);   
    
    // INPUTS

    protected abstract boolean getNoteDetectorValue();
}

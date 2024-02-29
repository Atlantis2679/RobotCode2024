package frc.robot.subsystems.loader.io;

import frc.lib.logfields.LogFieldsTable;

public class LoaderIOSim extends LoaderIO {
    public LoaderIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    public void setSpeed(double demandPrecentage) {
    }

    @Override
    protected boolean getNoteDetectorValue() {
        return false;
    }
}

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.loader.io.LoaderIO;
import frc.robot.subsystems.loader.io.LoaderIOSim;
import frc.robot.subsystems.loader.io.LoaderIOTalon;

public class Loader extends SubsystemBase {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final LoaderIO io = Robot.isSimulation()
            ? new LoaderIOSim(fieldsTable)
            : new LoaderIOTalon(fieldsTable);

    private boolean isNoteInside = false;

    public Loader() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
        fieldsTable.recordOutput("Is Note Inside", isNoteInside);
    }

    public void setSpeed(double demandPrecentage) {
        io.setSpeed(demandPrecentage);
        fieldsTable.recordOutput("Demand Speed", demandPrecentage);
    }

    public void setIsNoteInside(boolean isNoteInside) {
        this.isNoteInside = isNoteInside;
    }

    public boolean getIsNotInside() {
        return isNoteInside;
    }
}
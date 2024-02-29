package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Loader() {
        fieldsTable.update();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("loader note inside", getIsNoteInside());
        fieldsTable.recordOutput("Is Note Inside", getIsNoteInside());
    }

    public void setSpeed(double demandPrecentage) {
        io.setSpeed(demandPrecentage);
        fieldsTable.recordOutput("Demand Speed", demandPrecentage);
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : null);
    }

    public void stop() {
        setSpeed(0);
    }

    public boolean getIsNoteInside() {
        return io.noteDetectorvalue.getAsBoolean();
    }
}

package frc.robot.subsystems.flywheel.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class FlywheelIO extends IOBase {
    public final DoubleSupplier upperRollerSpeedRPS = fields.addDouble("upperRollerSpeedRPS",
            this::getUpperRollerSpeedRPS);
    public final DoubleSupplier lowerRollerSpeedRPS = fields.addDouble("lowerRollerSpeedRPS",
            this::getLowerRollerSpeedRPS);

    protected FlywheelIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getUpperRollerSpeedRPS();

    protected abstract double getLowerRollerSpeedRPS();

    // Outputs

    public abstract void setUpperRollerVoltage(double voltageDemand);

    public abstract void setLowerRollerVoltage(double voltageDemand);
}

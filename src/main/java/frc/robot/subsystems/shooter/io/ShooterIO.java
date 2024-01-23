package frc.robot.subsystems.shooter.io;

import java.util.function.DoubleSupplier;

import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class ShooterIO extends IOBase {
    public final DoubleSupplier upperRollerSpeedRPS = fields.addDouble("upperRollerSpeedRPS",
            this::getUpperRollerSpeedRPS);
    public final DoubleSupplier lowerRollerSpeedRPS = fields.addDouble("lowerRollerSpeedRPS",
            this::getLowerRollerSpeedRPS);

    protected ShooterIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    // inputs

    protected abstract double getUpperRollerSpeedRPS();

    protected abstract double getLowerRollerSpeedRPS();

    // Outputs

    public abstract void setUpperRollerSpeedVoltage(double voltageDemand);

    public abstract void setLowerRollerSpeedVoltage(double voltageDemand);
}

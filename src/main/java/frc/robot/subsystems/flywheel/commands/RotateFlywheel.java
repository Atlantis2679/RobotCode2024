package frc.robot.subsystems.flywheel.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;

import static frc.robot.subsystems.flywheel.FlywheelConstants.RotateFlywheel.*;

public class RotateFlywheel extends Command {
    private final Flywheel flywheel;

    private final DoubleSupplier upperRollerSpeedDemandSupplierRPS;
    private final DoubleSupplier lowerRollerSpeedDemandSupplierRPS;

    private final PIDController upperRollerSpeedPID = new PIDController(KP, KI, KD);
    private final PIDController lowerRollerSpeedPID = new PIDController(KP, KI, KD);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);

    public RotateFlywheel(
            Flywheel flywheel,
            DoubleSupplier upperRollerSpeedDemandSupplierRPS,
            DoubleSupplier lowerRollerSpeedDemandSupplierRPS) {
        this.flywheel = flywheel;
        this.upperRollerSpeedDemandSupplierRPS = upperRollerSpeedDemandSupplierRPS;
        this.lowerRollerSpeedDemandSupplierRPS = lowerRollerSpeedDemandSupplierRPS;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        upperRollerSpeedPID.reset();
        lowerRollerSpeedPID.reset();
    }

    @Override
    public void execute() {
        double upperRollerDemandSpeedRPS = upperRollerSpeedDemandSupplierRPS.getAsDouble();
        double upperRollerPIDResult = upperRollerSpeedPID.calculate(
                flywheel.getUpperRollerSpeedRPS(),
                upperRollerDemandSpeedRPS);
        double upperRollerFeedforwardResult = feedforward.calculate(upperRollerDemandSpeedRPS);
        double upperRollerVoltage = upperRollerPIDResult + upperRollerFeedforwardResult;

        double lowerRollerDemandSpeedRPS = lowerRollerSpeedDemandSupplierRPS.getAsDouble();
        double lowerRollerPIDResult = lowerRollerSpeedPID.calculate(
                flywheel.getLowerRollerSpeedRPS(),
                lowerRollerDemandSpeedRPS);
        double lowerRollerFeedforwardResult = feedforward.calculate(lowerRollerDemandSpeedRPS);
        double lowerRollerVoltage = lowerRollerPIDResult + lowerRollerFeedforwardResult;

        flywheel.setSpeed(upperRollerVoltage, lowerRollerVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

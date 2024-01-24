
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeController extends Command {

  private final Intake intake;
  private final DoubleSupplier angleDemandSupplier;

  public IntakeController(Intake intake, DoubleSupplier forwardDemandSupplier, DoubleSupplier backwardDemandSupplier) {
    this.intake = intake;
    this.angleDemandSupplier = backwardDemandSupplier;
    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.setAngleIntake(angleDemandSupplier.getAsDouble());

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

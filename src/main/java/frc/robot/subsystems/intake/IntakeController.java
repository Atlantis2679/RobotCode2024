
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeController extends Command {

  private final Intake intake;
  private final DoubleSupplier angleDemandSupplier;

  public IntakeController(Intake intake, DoubleSupplier angleDemandSupplier) {
    this.intake = intake;
    this.angleDemandSupplier = angleDemandSupplier;
    addRequirements(intake);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(angleDemandSupplier.getAsDouble() < 0){
      intake.setAngleIntake(angleDemandSupplier.getAsDouble() * INTAKE_CONTROLLER_MULTIPLIER );
    }

    if (angleDemandSupplier.getAsDouble() > 0) {
      intake.setAngleIntake(angleDemandSupplier.getAsDouble() * INTAKE_CONTROLLER_MULTIPLIER);
    }
  }

  @Override
  public void end(boolean interrupted) {
  } 

  @Override
  public boolean isFinished() {
    return false;
  }
}

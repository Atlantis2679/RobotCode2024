

package frc.robot.subsystems.intake;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class intakeButton extends Command {
  private final Intake intake;
  private final double intakeSpeed; 
  public intakeButton(Intake intake, double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setSpeedIntake(intakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setSpeedIntake(STOP_INTAKE_MOTOR);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

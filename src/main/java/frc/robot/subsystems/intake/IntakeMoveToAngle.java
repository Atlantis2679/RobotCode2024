
package frc.robot.subsystems.intake;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeMoveToAngle extends Command {
  private final Intake intake;
 private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
  IntakeConstants.MAX_VELOCITY,
  IntakeConstants.MAX_ACCELERATION));
  
private final Timer timer = new Timer();

  public IntakeMoveToAngle(Intake intake, double angle){
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    timer.start();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.calculateFeedforward(IntakeConstants.WANTED_ARM_ANGLE, IntakeConstants.MAX_VELOCITY, true);

    trapezoidProfile.calculate(timer.get(),
      new TrapezoidProfile.State(intake.getAbsoluteAngle(), intake.getArmSpeed()),
      new TrapezoidProfile.State(IntakeConstants.WANTED_ARM_ANGLE, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setAngleIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}

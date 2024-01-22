
package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;

public class Intake extends SubsystemBase {
  private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
  private final IntakeIO intakeIO = new IntakeIOSparkMax(fieldsTable);
  private final PIDController pidControllerIntake = new PIDController(
      IntakeConstants.KP,
      IntakeConstants.KI,
      IntakeConstants.KD);

  private final ArmFeedforward feedForwardIntake = new ArmFeedforward(
      IntakeConstants.KS,
      IntakeConstants.KG,
      IntakeConstants.KV,
      IntakeConstants.KA);


  public Intake() {
    fieldsTable.update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void setSpeedIntake(double intakeSpeed) {
  intakeIO.setIntakeSpeed(intakeSpeed);
  }
  
  public void setAngleIntake(double speedAngleIntake){
  intakeIO.setAngleIntake(speedAngleIntake);
  }


  public double getAbsoluteAngle(){
    return intakeIO.jointAngleDegrees.getAsDouble();
  }

  public double getIntakeSpeed(){
    return intakeIO.intakeSpeed.getAsDouble();
  }

  public double getArmSpeed(){
    return intakeIO.armSpeed.getAsDouble();
  }
  
  public double calculateFeedforward(double wantedArmAngle, double intakeArmVelocity, boolean usePID){

    double voltages = feedForwardIntake.calculate(Math.toRadians(wantedArmAngle), intakeArmVelocity);

      if (usePID) {
        voltages += pidControllerIntake.calculate(getAbsoluteAngle(),wantedArmAngle);
  }
  return voltages;

  }

  
}

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOSparkMax;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
  private final IntakeIO io = new IntakeIOSparkMax(fieldsTable);

  private final PIDController wristPidController = new PIDController(KP, KI, KD);
  SlewRateLimiter rollersSpeedLimiter = new SlewRateLimiter(ROLLERS_LIMIT_ACCELERATION_VOLTEG_PER_SECOND);
  SlewRateLimiter wristSpeedLimiter = new SlewRateLimiter(WRIST_LIMIT_ACCELERATION_VOLTEG_PER_SECOND);

  private final ArmFeedforward feedForwardIntake = new ArmFeedforward(KS, KG, KV, KA);

  public Intake() {
    fieldsTable.update();
  }

  @Override
  public void periodic() {
  }

  public void setSpeedIntake(double intakeSpeed) {
    intakeSpeed = rollersSpeedLimiter.calculate(intakeSpeed);
    io.setRollerSpeed(MathUtil.clamp(
      intakeSpeed,
      -ROLLER_VOLTEG,
       ROLLER_VOLTEG));
  }

  public void setWristSpeed(double jointSpeed) { 
    jointSpeed = wristSpeedLimiter.calculate(jointSpeed);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    io.setWristSpeed(MathUtil.clamp(
      jointSpeed,
      -ARM_VOLTEG,
       ARM_VOLTEG));
  }
  
  public double getAbsoluteAngle() {
    return io.wristAngleDegrees.getAsDouble();
  }

  public double getIntakeSpeed() {
    return io.rollersSpeed.getAsDouble();
  }

  public double getWristPercentageOutput() {
    return io.wristSpeed.getAsDouble();
  }
  public boolean getBeamBreakValue() {
    return io.beamBreakValue.getAsBoolean();
  }
  
  public double calculateFeedforward(double desiredWristAngleDegree, double wristVelocity, boolean usePID) {

    double voltages = feedForwardIntake.calculate(Math.toRadians(desiredWristAngleDegree), wristVelocity);

    if (usePID) {
      voltages += wristPidController.calculate(getAbsoluteAngle(), desiredWristAngleDegree);
    }
    return voltages;

  }

}
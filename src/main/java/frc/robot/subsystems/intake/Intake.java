
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
  private final PIDController pidControllerJoint = new PIDController(KP, KI, KD);
  SlewRateLimiter intakeSpeedLimiter = new SlewRateLimiter(INTAKE_LIMIT_ACCELERATION_VOLTEG_PER_SECOND);
  SlewRateLimiter jointSpeedLimiter = new SlewRateLimiter(JOINT_LIMIT_ACCELERATION_VOLTEG_PER_SECOND);

  private final ArmFeedforward feedForwardIntake = new ArmFeedforward(KS, KG, KV, KA);

  public Intake() {
    fieldsTable.update();
  }

  @Override
  public void periodic() {
  }

  public void setSpeedIntake(double intakeSpeed) {
    intakeSpeed = intakeSpeedLimiter.calculate(intakeSpeed);
    io.setIntakeSpeed(MathUtil.clamp(
      intakeSpeed,
      -INTAKE_LIMIT_ACCELERATION_VOLTEG_PER_SECOND,
       INTAKE_LIMIT_ACCELERATION_VOLTEG_PER_SECOND));
  }

  public void setAngleIntake(double jointSpeed) { 
    jointSpeed = jointSpeedLimiter.calculate(jointSpeed);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    io.setJointSpeed(MathUtil.clamp(
      jointSpeed,
      -JOINT_LIMIT_ACCELERATION_VOLTEG_PER_SECOND,
       JOINT_LIMIT_ACCELERATION_VOLTEG_PER_SECOND));
  }
  
  public double getAbsoluteAngle() {
    return io.jointAngleDegrees.getAsDouble();
  }

  public double getIntakeSpeed() {
    return io.intakeSpeed.getAsDouble();
  }

  public double getArmSpeed() {
    return io.jointSpeed.getAsDouble();
  }

  public double calculateFeedforward(double wantedArmAngle, double intakeArmVelocity, boolean usePID) {

    double voltages = feedForwardIntake.calculate(Math.toRadians(wantedArmAngle), intakeArmVelocity);

    if (usePID) {
      voltages += pidControllerJoint.calculate(getAbsoluteAngle(), wantedArmAngle);
    }
    return voltages;

  }

}
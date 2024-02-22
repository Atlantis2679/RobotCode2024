package frc.robot.allcommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.allcommands.AllCommandsConstants.CollectFromSource;
import frc.robot.allcommands.AllCommandsConstants.ScoreAmp;
import frc.robot.allcommands.AllCommandsConstants.ShootToSpeaker;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCMD;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.loader.LoaderCommands;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristCommands;

public class AllCommands {
    private final Swerve swerve;
    private final Flywheel flywheel;
    private final Pitcher pitcher;
    private final Loader loader;
    private final Intake intake;
    private final FlywheelCommands flywheelCMDs;
    private final PitcherCommands pitcherCMDs;
    private final LoaderCommands loaderCMDs;
    private final IntakeCommands intakeCMDs;

    private final ShootingCalculator shootingCalculator = new ShootingCalculator();

    public AllCommands(Swerve swerve, Flywheel flywheel, Pitcher pitcher, Loader loader, Intake intake) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.pitcher = pitcher;
        this.loader = loader;
        this.intake = intake;

        flywheelCMDs = new FlywheelCommands(flywheel);
        pitcherCMDs = new PitcherCommands(pitcher);
        loaderCMDs = new LoaderCommands(loader);
        intakeCMDs = new IntakeCommands(intake);

        swerve.registerCallbackOnPoseUpdate(shootingCalculator::update);
    }

    public Command shootToSpeaker() {
        return flywheelCMDs
                .spin(() -> shootingCalculator.getUpperRollerSpeedRPS(),
                        () -> shootingCalculator.getLowerRollerSpeedRPS())
                .alongWith(Commands
                        .waitUntil(() -> flywheel.atSpeed(
                                shootingCalculator.getUpperRollerSpeedRPS(),
                                shootingCalculator.getLowerRollerSpeedRPS()))
                        .andThen(loaderCMDs.release(ShootToSpeaker.SPEED_RELEASE)
                        .alongWith(Commands.waitSeconds(0.6)
                        .finallyDo(() -> flywheel.setSpeed(0, 0)))));
    }

    public Command scoreAmp() {
        return flywheelCMDs.spin(() -> ScoreAmp.UPPER_ROLLS_SPEED_RPS, () -> ScoreAmp.LOWER_ROLLS_SPEES_RPS)
            .alongWith(Commands
                .waitUntil(() -> flywheel.atSpeed(ScoreAmp.UPPER_ROLLS_SPEED_RPS, ScoreAmp.LOWER_ROLLS_SPEES_RPS))
                .andThen(loaderCMDs.release(ShootToSpeaker.SPEED_RELEASE).alongWith(Commands.waitSeconds(0.6)
                .finallyDo(() -> {
                    loader.setSpeed(0);
                    flywheel.setSpeed(0, 0);
                }))));
    }

    public Command collectFromSource() {
        return pitcherCMDs.adjustToAngle(CollectFromSource.DEGREES_SOURCE).andThen(flywheelCMDs.spin(() -> CollectFromSource.UPPER_ROLLS_SPEED_RPS, () -> CollectFromSource.LOWER_ROLLS_SPEES_RPS)
        .alongWith(Commands
                .waitUntil(() -> loader.getIsNoteInside())
                .andThen(loaderCMDs.release(CollectFromSource.BRING_BACK_NOTE_TO_SHOOTER))
                    .alongWith(Commands
                        .waitUntil(() -> loader.getIsNoteInside())
                        .finallyDo(() -> loader.setSpeed(0)))));
    }

    public Command manualShooter(DoubleSupplier pitcherSupplier, DoubleSupplier loaderSupplier,
        DoubleSupplier upperRollSupplier, DoubleSupplier lowerRollSupplier) {

        return pitcherCMDs.manualController(pitcherSupplier)
            .alongWith(loaderCMDs.manualController(loaderSupplier))
            .alongWith(flywheelCMDs.manualController(upperRollSupplier, lowerRollSupplier));
    }
  
   public Command openIntake() {
                return wristCMDs.moveToAngle(Open.COLLECTING_WRIST_ANGLE_DEGREE)
                .andThen(Commands.waitUntil(() -> wrist.getAbsoluteAngleDegrees() < Open.START_GRIPPER_WRIST_ANGLE_DEGREE))
                .andThen(gripperCMD.spin(Open.START_GRIPPER_WRIST_ANGLE_DEGREE));
    }

    public Command closeIntake() {
            return wristCMDs.moveToAngle(Close.CLOSED_WRIST_ANGLE_DEGREE);
    }

    public Command manualIntake(DoubleSupplier wristSpeed, DoubleSupplier gripperSpeed) {
            return wristCMDs.manualController(wristSpeed).alongWith(gripperCMD.manualController(gripperSpeed));
    }
}

package frc.robot.allcommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// import frc.robot.allcommands.AllCommandsConstants.CollectFromSource;
// import frc.robot.allcommands.AllCommandsConstants.ScoreAmp;
// import frc.robot.allcommands.AllCommandsConstants.ShootToSpeaker;
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
        private final Wrist wrist;
        private final Gripper gripper;
        private final FlywheelCommands flywheelCMDs;
        private final PitcherCommands pitcherCMDs;
        private final LoaderCommands loaderCMDs;
        private final WristCommands wristCMDs;
        private final GripperCMD gripperCMD;
        private final ShootingCalculator shootingCalculator = new ShootingCalculator();

        public AllCommands(Swerve swerve, Flywheel flywheel, Pitcher pitcher, Loader loader, Wrist wrist,
                        Gripper gripper) {
                this.swerve = swerve;
                this.flywheel = flywheel;
                this.pitcher = pitcher;
                this.loader = loader;
                this.wrist = wrist;
                this.gripper = gripper;

                flywheelCMDs = new FlywheelCommands(flywheel);
                pitcherCMDs = new PitcherCommands(pitcher);
                loaderCMDs = new LoaderCommands(loader);
                wristCMDs = new WristCommands(wrist);
                gripperCMD = new GripperCMD(gripper);
                swerve.registerCallbackOnPoseUpdate(shootingCalculator::update);
        }

        public Command shootToSpeaker() {
                return Commands.waitUntil(() -> flywheel.atSpeed(
                                shootingCalculator.getUpperRollerSpeedRPS(),
                                shootingCalculator.getLowerRollerSpeedRPS()))
                                .andThen(loaderCMDs.spin(ShootToSpeaker.SPEED_RELEASE)
                                                .alongWith(Commands.waitSeconds(0.6)
                                                                .finallyDo(() -> flywheel.setSpeed(0,
                                                                                0))));
        }

        public Command scoreAmp() {
                return flywheelCMDs.spin(() -> ScoreAmp.UPPER_ROLLS_SPEED_RPS, () -> ScoreAmp.LOWER_ROLLS_SPEES_RPS)
                                .alongWith(Commands
                                                .waitUntil(() -> flywheel.atSpeed(ScoreAmp.UPPER_ROLLS_SPEED_RPS,
                                                                ScoreAmp.LOWER_ROLLS_SPEES_RPS))
                                                .andThen(loaderCMDs.spin(ScoreAmp.SPEED_RELEASE)));
        }

        public Command collectFromSource() {
                return pitcherCMDs.adjustToAngle(CollectFromSource.DEGREES_SOURCE).andThen(flywheelCMDs
                                .spin(() -> CollectFromSource.UPPER_ROLLS_SPEED_RPS,
                                                () -> CollectFromSource.LOWER_ROLLS_SPEES_RPS)
                                .alongWith(Commands
                                                .waitUntil(loader::getIsNoteInside)
                                                .andThen(loaderCMDs
                                                                .spin(CollectFromSource.BRING_BACK_NOTE_TO_SHOOTER))
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
                                .andThen(Commands.waitUntil(() -> wrist
                                                .getAbsoluteAngleDegrees() < Open.START_GRIPPER_WRIST_ANGLE_DEGREE)
                                                .andThen(gripperCMD.spin(Open.GRIPPER_COLLECTING_SPEED)));
        }

        public Command closeIntake() {
                return wristCMDs.moveToAngle(Close.CLOSED_WRIST_ANGLE_DEGREE);
        }

        public Command manualIntake(DoubleSupplier wristSpeed, DoubleSupplier gripperSpeed) {
                return wristCMDs.manualController(wristSpeed).alongWith(gripperCMD.manualController(gripperSpeed));
        }

        public Command handOff() {
                return wristCMDs.moveToAngle(HandOff.WRIST_HANDOFF_ANGLE_DEGRRES)
                                .alongWith(pitcherCMDs.adjustToAngle(HandOff.PITCHER_HANDOF_ANGLE_DEGRRES))
                                .alongWith(Commands.waitUntil(() -> wrist
                                                .getAbsoluteAngleDegrees() > HandOff.WRIST_STARTING_LOADER_ANGLE_DEGRRE))
                                .andThen(loaderCMDs.spin(HandOff.LOADER_HANDOFF_PRECENTAGE_OUTPUT))
                                .alongWith(Commands.waitUntil(() -> wrist.isAtAngle(wrist.getAbsoluteAngleDegrees()))
                                                .andThen(gripperCMD.spin(HandOff.GRIPPER_HANDOFF_SPEED))
                                                .alongWith(Commands.waitUntil(loader::getIsNoteInside))
                                                .finallyDo(() -> {
                                                        gripper.stop();
                                                        loader.stop();
                                                }));

        }
}

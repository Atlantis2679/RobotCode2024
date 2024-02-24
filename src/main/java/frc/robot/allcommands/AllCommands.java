package frc.robot.allcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.allcommands.AllCommandsConstants.Close;
import frc.robot.allcommands.AllCommandsConstants.CollectFromSource;
import frc.robot.allcommands.AllCommandsConstants.Handoff;
import frc.robot.allcommands.AllCommandsConstants.OpenIntake;
import frc.robot.allcommands.AllCommandsConstants.ReadyToShootToAmp;
import frc.robot.allcommands.AllCommandsConstants.ReadyToShootToSpeaker;
import frc.robot.allcommands.AllCommandsConstants.ShootToSpeaker;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
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
        private final GripperCommands gripperCMD;
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
                gripperCMD = new GripperCommands(gripper);
                swerve.registerCallbackOnPoseUpdate(shootingCalculator::update);
        }

        public Command pitcherReadyToHandOff() {
                return pitcherCMDs.adjustToAngle(Handoff.PITCHER_DEGRRES)
                                .withName("pitcherReadyToHandoff");
        }

        public Command pitcherWithNoteIdle() {
                return pitcherCMDs.adjustToAngle(ReadyToShootToSpeaker.PITCHER_DEGREES)
                                .withName("readyToShootToSpeaker");
        }

        public Command readyToShootToSpeaker() {
                return Commands.parallel(
                                flywheelCMDs.spin(
                                                ReadyToShootToSpeaker.UPPER_ROLLERS_SPEED,
                                                ReadyToShootToSpeaker.LOWER_ROLLERS_SPEED),
                                pitcherCMDs.adjustToAngle(ReadyToShootToSpeaker.PITCHER_DEGREES))
                                .withName("readyToShootToSpeaker");
        }

        public Command readyToShootToAmp() {
                return Commands.parallel(
                                flywheelCMDs.spin(
                                                ReadyToShootToAmp.UPPER_ROLLERS_SPEED,
                                                ReadyToShootToAmp.LOWER_ROLLERS_SPEED),
                                pitcherCMDs.adjustToAngle(ReadyToShootToAmp.PITCHER_DEGREES))
                                .withName("readyToShootToAmp");
        }

        public Command shoot() {
                return Commands.waitUntil(flywheel::atSpeed)
                                .andThen(loaderCMDs.spin(ShootToSpeaker.SPEED_RELEASE)).withName("shoot");
        }

        public Command collectFromSource() {
                return Commands.parallel(
                                pitcherCMDs.adjustToAngle(CollectFromSource.PITCHER_DEGREES),
                                flywheelCMDs.spin(
                                                () -> CollectFromSource.UPPER_ROLLS_SPEED_RPS,
                                                () -> CollectFromSource.LOWER_ROLLS_SPEES_RPS)
                                                .raceWith(Commands.waitUntil(loader::getIsNoteInside).andThen(
                                                                Commands.waitUntil(() -> !loader.getIsNoteInside()))),
                                Commands.waitUntil(loader::getIsNoteInside)
                                                .andThen(loaderCMDs.spin(CollectFromSource.LOADER_SPEED_TO_INSIDE)
                                                                .until(() -> !loader.getIsNoteInside()))
                                                .andThen(loaderCMDs.spin(
                                                                CollectFromSource.BRING_BACK_NOTE_TO_SHOOTER_LOADER_SPEED)
                                                                .until(loader::getIsNoteInside)))
                                .withName("collectFromSource");
        }

        public Command openIntake() {
                return Commands.parallel(wristCMDs.moveToAngle(OpenIntake.COLLECTING_WRIST_ANGLE_DEGREE),
                                runWhen(() -> wrist
                                                .getAbsoluteAngleDegrees() < OpenIntake.START_GRIPPER_WRIST_ANGLE_DEGREE,
                                                gripperCMD.spin(OpenIntake.GRIPPER_COLLECTING_SPEED)))
                                .until(gripper::getIsNoteInside)
                                .withName("openIntake");
        }

        public Command closeWrist() {
                return wristCMDs.moveToAngle(Close.CLOSED_WRIST_ANGLE_DEGREE).withName("closeIntake");
        }

        public Command handoff() {
                return Commands.parallel(
                                wristCMDs.moveToAngle(Handoff.WRIST_HANDOFF_ANGLE_DEGRRES),
                                pitcherCMDs.adjustToAngle(Handoff.PITCHER_DEGRRES),
                                runWhen(() -> wrist
                                                .getAbsoluteAngleDegrees() < Handoff.WRIST_STARTING_LOADER_ANGLE_DEGRRE,
                                                loaderCMDs.spin(Handoff.LOADER_HANDOFF_PRECENTAGE_OUTPUT)),
                                runWhen(() -> wrist.isAtAngle(Handoff.WRIST_HANDOFF_ANGLE_DEGRRES),
                                                gripperCMD.spin(Handoff.GRIPPER_HANDOFF_SPEED)))
                                .until(loader::getIsNoteInside).withName("handoff");
        }

        public Command manualIntake(DoubleSupplier wristSpeed, DoubleSupplier gripperSpeed) {
                return Commands.parallel(
                                wristCMDs.manualController(wristSpeed),
                                gripperCMD.manualController(gripperSpeed))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .withName("manualIntake");
        }

        public Command manualShooter(DoubleSupplier pitcherSupplier, DoubleSupplier loaderSupplier,
                        DoubleSupplier upperRollSupplier, DoubleSupplier lowerRollSupplier) {

                return Commands.parallel(
                                pitcherCMDs.manualController(pitcherSupplier),
                                loaderCMDs.manualController(loaderSupplier),
                                flywheelCMDs.manualController(upperRollSupplier, lowerRollSupplier))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .withName("manualShooter");
        }

        public Command stopAll() {
                return Commands.runOnce(() -> {
                        pitcher.stop();
                        wrist.stop();
                        gripper.stop();
                        flywheel.stop();
                        loader.stop();
                        swerve.stop();
                }, pitcher, wrist, gripper, flywheel, loader, swerve)
                                .ignoringDisable(true)
                                .withName("stopAll");
        }

        private Command runWhen(BooleanSupplier condition, Command command) {
                return Commands.waitUntil(condition).andThen(command);
        }
}

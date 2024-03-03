package frc.robot.allcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.tuneables.extensions.TuneableWrapperCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.allcommands.AllCommandsConstants.Close;
import frc.robot.allcommands.AllCommandsConstants.Deliver;
import frc.robot.allcommands.AllCommandsConstants.Eat;
import frc.robot.allcommands.AllCommandsConstants.GetReadyToScoreAMP;
import frc.robot.allcommands.AllCommandsConstants.OpenIntake;
import frc.robot.allcommands.AllCommandsConstants.ScoreAmp;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.gripper.io.GripperIO;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristCommands;

public class AllCommands {
        private final Swerve swerve;
        private final Wrist wrist;
        private final Gripper gripper;
        private final Elevator elevator;
        private final WristCommands wristCMDs;
        private final GripperCommands gripperCMD;
        private final ElevatorCommands elevatorCMD;
        private double counter;

        private final ShootingCalculator shootingCalculator = new ShootingCalculator();

        public AllCommands(Swerve swerve, Wrist wrist, Gripper gripper, Elevator elevator) {
                this.swerve = swerve;
                this.wrist = wrist;
                this.elevator = elevator;
                this.gripper = gripper;

                wristCMDs = new WristCommands(wrist);
                gripperCMD = new GripperCommands(gripper);
                elevatorCMD = new ElevatorCommands(elevator);
                swerve.registerCallbackOnPoseUpdate(shootingCalculator::update);

                this.counter = 0;
        }

        public TuneableCommand scoreAMPTuenble() {
                return TuneableWrapperCommand.wrap((table) -> {
                        DoubleHolder upperRollerRPS = table.addNumber("upper rollers RPS",
                                        -0.35);
                        DoubleHolder lowerRollerRPS = table.addNumber("lower rollers RPS",
                                        0.5);
                        return Commands.parallel(
                                        gripperCMD.spin(upperRollerRPS.get(),
                                                        lowerRollerRPS.get()))
                                        .withName("readyToShootTuneable");
                });
        }

        public Command scoreAMP() {
                return Commands.parallel(
                                runWhen(() -> wrist.isAtAngle(ScoreAmp.AMP_DEGREES + counter),
                                                gripperCMD.spin(ScoreAmp.UPPER_ROLLS_SPEED_RPS,
                                                                ScoreAmp.LOWER_ROLLS_SPEES_RPS)))
                                .withName("scoreAMP");
        }

        // public Command scoreAMP() {
        // return gripperCMD.spin(ScoreAmp.UPPER_ROLLS_SPEED_RPS,
        // ScoreAmp.LOWER_ROLLS_SPEES_RPS)
        // .withName("scoreAMP");
        // }

        public Command getReadyToScoreAMP() {
                return wristCMDs.moveToAngle(GetReadyToScoreAMP.AMP_DEGREES + counter)
                                .alongWith(gripperCMD.spin(-GetReadyToScoreAMP.KEEPING_NOTE_INSIDE_GRIPPER_SPEED_RPS,
                                                GetReadyToScoreAMP.KEEPING_NOTE_INSIDE_GRIPPER_SPEED_RPS))
                                .withName("getReadyToScoreAMP");
        }

        public Command openIntake() {
                return Commands.parallel(wristCMDs.moveToAngle(OpenIntake.COLLECTING_WRIST_ANGLE_DEGREE),
                                runWhen(() -> wrist
                                                .getAbsoluteAngleDegrees() < OpenIntake.START_GRIPPER_WRIST_ANGLE_DEGREE,
                                                gripperCMD.spin(OpenIntake.UPPER_GRIPPER_COLLECTING_SPEED,
                                                                OpenIntake.LOWER_GRIPPER_COLLECTING_SPEED)))
                                .until(gripper::getIsNoteInside)
                                .withName("openIntake");
        }

        public Command closeWrist() {
                return wristCMDs.moveToAngle(Close.CLOSED_WRIST_ANGLE_DEGREE).withName("closeIntake");
        }

        public Command deliver() {
                return Commands.parallel(
                                wristCMDs.moveToAngle(Deliver.DELIVERY_WRIST_ANGLE_DEGREE),
                                runWhen(() -> wrist
                                                .getAbsoluteAngleDegrees() < Deliver.START_GRIPPER_WRIST_ANGLE_DEGREE,
                                                gripperCMD.spin(-Deliver.DELIVERY_GRIPPERS_SPEED_RPS,
                                                                Deliver.DELIVERY_GRIPPERS_SPEED_RPS)));
        }

        public Command eat() {
                return gripperCMD.spin(Eat.GRIPPER_EATING_SPEED_RPS, -Eat.GRIPPER_EATING_SPEED_RPS);

        }

                public Command puke() {
                return gripperCMD.spin(-Eat.GRIPPER_EATING_SPEED_RPS, Eat.GRIPPER_EATING_SPEED_RPS);

        }

        public Command manualIntake(DoubleSupplier wristSpeed, DoubleSupplier upperGripperSpeed,
                        DoubleSupplier lowerGripperSpeed) {
                return Commands.parallel(
                                wristCMDs.manualController(wristSpeed),
                                gripperCMD.manualController(upperGripperSpeed, lowerGripperSpeed))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                                .withName("manualIntake");
        }

        public Command manualElevator(DoubleSupplier elevatorSpeed, BooleanSupplier isNegative) {
                return Commands.parallel(
                                closeWrist(),
                                elevatorCMD.manualControl(elevatorSpeed, isNegative)).withName("manualElevator");
        }

        public Command changeCounter(BooleanSupplier isPlus) {
                return Commands.run(() -> counter = isPlus.getAsBoolean() ? counter + 0.5 : counter - 0.5);
        }

        public Command stopAll() {
                return Commands.runOnce(() -> {
                        wrist.stop();
                        gripper.stop();
                        swerve.stop();
                        elevator.stop();
                }, wrist, gripper, swerve, elevator)
                                .ignoringDisable(true)
                                .withName("stopAll");
        }

        private Command runWhen(BooleanSupplier condition, Command command) {
                return Commands.waitUntil(condition).andThen(command);
        }
}

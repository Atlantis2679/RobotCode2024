package frc.robot.allcommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.allcommands.AllCommandsConstants.Close;
import frc.robot.allcommands.AllCommandsConstants.Open;
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

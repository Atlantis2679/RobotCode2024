package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.allcommands.AllCommands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.loader.LoaderCommands;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Pitcher pitcher = new Pitcher();
    private final Wrist wrist = new Wrist();
    private final Gripper gripper = new Gripper();
    private final Flywheel flywheel = new Flywheel();
    private final Loader loader = new Loader();

    private final NaturalXboxController driverController = new
    NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERTATOR_PORT);

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    // private final PitcherCommands pitcherCommands = new PitcherCommands(pitcher);
    // private final IntakeCommands intakeCommands = new IntakeCommands(intake);
        private final AllCommands allCommands = new AllCommands(swerve, flywheel, pitcher, loader, wrist, gripper);
    public RobotContainer() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        TuneableCommand driveCommand = swerveCommands.controller(
        driverController::getLeftY,
        driverController::getLeftX,
        driverController::getRightX,
        driverController.leftBumper().negate()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());
        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));
        driverController.y().onTrue(swerveCommands.xWheelLock());

        TuneablesManager.add("Swerve/modules control mode",
        swerveCommands.controlModules(
        driverController::getLeftX,
        driverController::getLeftY,
        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        wrist.setDefaultCommand(allCommands.closeIntake());
        operatorController.a().whileTrue(allCommands.openIntake());
        operatorController.b().whileTrue(allCommands.shootToSpeaker());
    }

    public void stopAll(){
        pitcher.stop();
        wrist.stop();
        gripper.stop();
        flywheel.stop();
        loader.stop();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

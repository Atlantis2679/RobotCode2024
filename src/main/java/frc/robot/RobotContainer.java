package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Pitcher pitcher = new Pitcher();
    private final Intake intake = new Intake();

    private final NaturalXboxController driverController = new NaturalXboxController(RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERTATOR_PORT);
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final PitcherCommands pitcherCommands = new PitcherCommands(pitcher);
    private final IntakeCommands intakeCommands = new IntakeCommands(intake);
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

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightX).fullTuneable());
    }

    private void configureOperatorBindings() {
        intake.setDefaultCommand(intakeCommands.close());
        operatorController.leftBumper().onTrue(intakeCommands.open());
        operatorController.rightBumper().whileTrue(intakeCommands.manualController(operatorController::getRightY));

        operatorController.a().onTrue(pitcherCommands.adjustToAngle(90));
        operatorController.y().onTrue(pitcherCommands.adjustToAngle(0));
        operatorController.b().whileTrue(pitcherCommands.adjustToAngle(() -> operatorController.getLeftY() * 90));
        pitcher.setDefaultCommand(pitcherCommands.manualController(() -> operatorController.getLeftY()));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

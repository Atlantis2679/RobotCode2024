package frc.robot.allcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.loader.LoaderCommands;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;
import frc.robot.subsystems.swerve.Swerve;

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

    public Command shoot() {
        return flywheelCMDs
                .rotate(() -> shootingCalculator.getUpperRollerSpeedRPS(),
                        () -> shootingCalculator.getLowerRollerSpeedRPS())
                .alongWith(Commands
                        .waitUntil(() -> flywheel.atSpeed(
                                shootingCalculator.getUpperRollerSpeedRPS(),
                                shootingCalculator.getLowerRollerSpeedRPS()))
                        .andThen(loaderCMDs.release()));
    }
}

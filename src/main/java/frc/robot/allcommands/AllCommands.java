package frc.robot.allcommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.loader.LoaderCommands;
import frc.robot.subsystems.pitcher.Pitcher;
import frc.robot.subsystems.pitcher.PitcherCommands;

public class AllCommands {
    private final Flywheel flywheel;
    private final Pitcher pitcher;
    private final Loader loader;
    private final Intake intake;
    private final FlywheelCommands flywheelCMDs;
    private final PitcherCommands pitcherCMDs;
    private final LoaderCommands loaderCMDs;
    private final IntakeCommands intakeCMDs;

    public AllCommands(Flywheel flywheel, Pitcher pitcher, Loader loader, Intake intake) {
        this.flywheel = flywheel;
        this.pitcher = pitcher;
        this.loader = loader;
        this.intake = intake;

        flywheelCMDs = new FlywheelCommands(flywheel);
        pitcherCMDs = new PitcherCommands(pitcher);
        loaderCMDs = new LoaderCommands(loader);
        intakeCMDs = new IntakeCommands(intake);
    }

    public Command shoot() {
        double upperRollerSpeed = 3;
        double lowerRollerSpeed = 4;
        return flywheelCMDs.rotate(() -> upperRollerSpeed, () -> lowerRollerSpeed)
                .alongWith(Commands.waitUntil(() -> flywheel.atSpeed(upperRollerSpeed, lowerRollerSpeed)).andThen(loaderCMDs.release()));
    }
}

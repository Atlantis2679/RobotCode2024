package frc.robot.subsystems.pitcher;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logfields.LogFieldsTable;

public class PitcherVisualizer {
    private final LogFieldsTable fieldsTable;
    private final String name;

    private final Mechanism2d mech2d;
    private final MechanismRoot2d root;
    private final MechanismLigament2d rightPlate;
    private final MechanismLigament2d leftPlate;

    public PitcherVisualizer(LogFieldsTable fieldsTable, String name, Color8Bit color) {
        this.fieldsTable = fieldsTable;
        this.name = name;

        mech2d = new Mechanism2d(1, 1);

        root = mech2d.getRoot("PitcherPivot", 0.5, 0.4);

        rightPlate = root.append(
                new MechanismLigament2d("RightPitcherPlate", 0.4, 0, 10, color));

        leftPlate = root.append(
                new MechanismLigament2d("LeftPitcherPlate", 0.3, 180, 10, color));
    }

    public void update(double angleDegrees) {
        rightPlate.setAngle(angleDegrees);
        leftPlate.setAngle(angleDegrees + 180);
        fieldsTable.recordOutput(name, mech2d);
    }
}

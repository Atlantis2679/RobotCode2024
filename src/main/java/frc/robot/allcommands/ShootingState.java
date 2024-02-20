package frc.robot.allcommands;

public class ShootingState {
    public final double distanceFromTarget;
    public final double pitcherAngleDegrees;
    public final double upperRollerRPS;
    public final double lowerRollerRPS;

    public ShootingState(double distanceFromTarget, double pitcherAngleDegrees, double upperRollerRPS, double lowerRollerRPS) {
        this.distanceFromTarget = distanceFromTarget;
        this.pitcherAngleDegrees = pitcherAngleDegrees;
        this.upperRollerRPS = upperRollerRPS;
        this.lowerRollerRPS = lowerRollerRPS;
    }
}

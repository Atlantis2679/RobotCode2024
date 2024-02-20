package frc.robot.allcommands;

import frc.robot.utils.LinearInterpolation;
import static frc.robot.allcommands.ShootingMeasurments.*;
import static frc.robot.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootingCalculator {
    private final LinearInterpolation pitcherAngleDegreesLinearInterpolation;
    private final LinearInterpolation upperRollerSpeedRPSLinearInterpolation;
    private final LinearInterpolation lowerRollerSpeedRPSLinearInterpolation;

    private double pitcherAngleDegrees;
    private double upperRollerSpeedRPS;
    private double lowerRollerSpeedRPS;

    public ShootingCalculator() {
        List<LinearInterpolation.Point> pitcherAngleDegreesPoints = new ArrayList<>();
        List<LinearInterpolation.Point> upperRollerSpeedRPSPoints = new ArrayList<>();
        List<LinearInterpolation.Point> lowerRollerSpeedRPSPoints = new ArrayList<>();
        for (ShootingState shootingState : ALL_MEASURMENTS) {
            pitcherAngleDegreesPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget, shootingState.pitcherAngleDegrees));
            upperRollerSpeedRPSPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget, shootingState.upperRollerRPS));
            lowerRollerSpeedRPSPoints.add(
                    new LinearInterpolation.Point(shootingState.distanceFromTarget, shootingState.lowerRollerRPS));
        }
        pitcherAngleDegreesLinearInterpolation = new LinearInterpolation(pitcherAngleDegreesPoints);
        upperRollerSpeedRPSLinearInterpolation = new LinearInterpolation(upperRollerSpeedRPSPoints);
        lowerRollerSpeedRPSLinearInterpolation = new LinearInterpolation(pitcherAngleDegreesPoints);
    }

    public void update(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d speakerPose = isRedAlliance ? RED_SPEAKER_POSE : BLUE_SPEAKER_POSE;
        double distanceFromTarget = Math.sqrt(
                Math.pow(robotPose.getX() - speakerPose.getX(), 2)
                        + Math.pow(robotPose.getY() - speakerPose.getY(), 2));

        pitcherAngleDegrees = pitcherAngleDegreesLinearInterpolation.calculate(distanceFromTarget);
        upperRollerSpeedRPS = upperRollerSpeedRPSLinearInterpolation.calculate(distanceFromTarget);
        lowerRollerSpeedRPS = lowerRollerSpeedRPSLinearInterpolation.calculate(distanceFromTarget);
    }

    public double getPitcherAngleDegrees() {
        return pitcherAngleDegrees;
    }

    public double getUpperRollerSpeedRPS() {
        return upperRollerSpeedRPS;
    }

    public double getLowerRollerSpeedRPS() {
        return lowerRollerSpeedRPS;
    }
}

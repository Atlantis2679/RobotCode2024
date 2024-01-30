package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalSensorHelper {
    private double measuredAngleDegreesCW = 0;
    private double offsetDegreesCW = 0;

    public RotationalSensorHelper(double initialMeasurementDegreesCW, double initialOffsetDegreesCW) {
        measuredAngleDegreesCW = initialMeasurementDegreesCW;
        offsetDegreesCW = initialOffsetDegreesCW;
    }

    public RotationalSensorHelper(double initialMeasurementDegreesCW) {
        this(initialMeasurementDegreesCW, 0);
    }

    public void update(double measuredAngleDegreesCW) {
        this.measuredAngleDegreesCW = measuredAngleDegreesCW;
    }

    public double getRawMeasuredAngleDegreesCW() {
        return measuredAngleDegreesCW;
    }

    public double getRawMeasuredAngleDegreesCCW() {
        return -measuredAngleDegreesCW;
    }

    public Rotation2d getRawMeasuredAngleCW() {
        return Rotation2d.fromDegrees(measuredAngleDegreesCW);
    }

    public Rotation2d getRawMeasuredAngleCCW() {
        return Rotation2d.fromDegrees(-measuredAngleDegreesCW);
    }

    public double getAngleDegreesCW() {
        return measuredAngleDegreesCW - offsetDegreesCW;
    }

    public double getAngleDegreesCCW() {
        return -getAngleDegreesCW();
    }

    public Rotation2d getAngleCW() {
        return Rotation2d.fromDegrees(getAngleDegreesCW());
    }

    public Rotation2d getAngleCCW() {
        return Rotation2d.fromDegrees(getAngleDegreesCCW());
    }

    public void setOffsetDegreesCW(double offsetDegreesCW) {
        this.offsetDegreesCW = offsetDegreesCW;
    }

    public double getOffsetDegreesCW() {
        return offsetDegreesCW;
    }

    public void setAngleDegreesCW(double angleDegreesCW) {
        offsetDegreesCW = measuredAngleDegreesCW - angleDegreesCW;
    }

    public void setAngleCW(Rotation2d angleCW) {
        setAngleDegreesCW(angleCW.getDegrees());
    }

    public void setAngleDegreesCCW(double angleDegreesCCW) {
        setAngleDegreesCW(-angleDegreesCCW);
    }

    public void setAngleCCW(Rotation2d angleCCW) {
        setAngleDegreesCCW(angleCCW.getDegrees());
    }
}

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalSensorHelper {
    private Rotation2d measuredAngle;
    private Rotation2d offset;

    private boolean continousWrapEnabled;
    private Rotation2d continousWrapUpperBound;
    private Rotation2d continousWrapLowerBound;

    public RotationalSensorHelper(Rotation2d initialMeasuredAngle, Rotation2d initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
    }

    public RotationalSensorHelper(Rotation2d initalMeasuredAngle) {
        this(initalMeasuredAngle, new Rotation2d(0));
    }

    public void update(Rotation2d measuredAngle) {
        this.measuredAngle = measuredAngle;
    }

    public Rotation2d getMeasuredAngle() {
        return measuredAngle;
    }

    public Rotation2d getAngle() {
        Rotation2d angle = measuredAngle.minus(offset);
        if (continousWrapEnabled) {
            while (angle.getRadians() > continousWrapUpperBound.getRadians()) {
                angle = angle.minus(Rotation2d.fromRotations(1));
            }
            while(angle.getRadians() < continousWrapLowerBound.getRadians()) {
                angle = angle.plus(Rotation2d.fromRotations(1));
            }
        }
        return angle;
    }

    public void resetAngle(Rotation2d newAngle) {
        offset = measuredAngle.minus(newAngle);
    }

    public void enableContinousWrap(Rotation2d upperBound) {
        continousWrapEnabled = true;
        continousWrapUpperBound = upperBound;
        continousWrapLowerBound = upperBound.minus(Rotation2d.fromRotations(1));
    }

    public void setOffset(Rotation2d offset) {
        this.offset = offset;
    }

    public Rotation2d getOffset() {
        return offset;
    }
}

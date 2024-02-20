package frc.robot.utils;

public class PrimitiveRotationalSensorHelper {
    private double measuredAngle;
    private double offset;

    private boolean continousWrapEnabled;
    private double continousWrapUpperBound;
    private double continousWrapLowerBound;
    private double fullRotation;

    public PrimitiveRotationalSensorHelper(double initialMeasuredAngle, double initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
    }

    public PrimitiveRotationalSensorHelper(double initalMeasuredAngle) {
        this(initalMeasuredAngle, 0);
    }

    public void update(double measuredAngle) {
        this.measuredAngle = measuredAngle;
    }

    public double getMeasuredAngle() {
        return measuredAngle;
    }

    public double getAngle() {
        double angle = measuredAngle - offset;
        if (continousWrapEnabled) {
            while (angle > continousWrapUpperBound) {
                angle -= fullRotation;
            }
            while(angle < continousWrapLowerBound) {
                angle += fullRotation;
            }
        }
        return angle;
    }

    public void resetAngle(double newAngle) {
        offset = measuredAngle - newAngle;
    }

    public void enableContinousWrap(double upperBound, double fullRotation) {
        continousWrapEnabled = true;
        this.fullRotation = fullRotation;
        continousWrapUpperBound = upperBound;
        continousWrapLowerBound = upperBound - fullRotation;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return offset;
    }
}

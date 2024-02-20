package frc.robot.utils;

import java.util.List;

public class LinearInterpolation {
    private final List<Point> points;

    public LinearInterpolation(List<Point> points) {
        this.points = points;
    }

    public double calculate(double x) {
        for (int i = 0; i < points.size() - 1; i++) {
            if (points.get(i).x <= x) {
                Point startPoint = points.get(i);
                Point endPoint = points.get(i + 1);

                return ((x - startPoint.x) / (endPoint.x - startPoint.x))
                        * (endPoint.y - startPoint.y)
                        + startPoint.y;
            }
        }
        return points.get(points.size() - 1).y;
    }

    public static class Point {
        public final double x;
        public final double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}

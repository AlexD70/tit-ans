import titans.geometry.Point2d;
import titans.roads.Spline;

public class RoadResources {
    public static class MathTests {
        public static final Point2d[] points = {new Point2d(0.732, 2.901), new Point2d(-5.17, 3.3),
                new Point2d(4.87, -5), new Point2d(8.3, 18.33),
                new Point2d(-0.233, 1.5), new Point2d(-21, 3)
        };

        public static final Spline testSpline = Spline.buildSpline6(points[0], points[1], points[2], points[3], points[4], points[5]);
        public static final Spline testSpline2 = Spline.buildSpline6(points[3], points[4], points[2], points[5], points[0], points[1]);
        public static final long MAX_MILLIS_INTEGRATION = 10, MAX_MILLIS_BRENTOPT = 20;
    }
}

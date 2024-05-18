import org.junit.jupiter.api.Test;
import titans.algebra.NPoly;
import titans.geometry.Point2d;
import titans.roads.Road;
import titans.roads.Spline;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import java.util.Arrays;

import org.apache.commons.math3.util.Pair;

public class RoadTests {
    static NPoly c1x = new NPoly(5), c1y = new NPoly(5);
    static NPoly c2x = new NPoly(5), c2y = new NPoly(5);
    static {
        c1x.assignCoefficients(0, 0, 0, 0, 1, -2.3);
        c1y.assignCoefficients(0, 0, 0, -4, 1, 4);
        c2x.assignCoefficients(1.2, 0.1, 3, 1, -1, -1.3);
        c2y.assignCoefficients(0.5, 4, -1, 4, 0.2, 1);
    }
    static final double lenc1 = 3.4023;
    static final double MAX_LEN_ERR = 0.01;
    Road road = new Road();

    public void buildTestRoad() throws IllegalAccessException, NoSuchFieldException {
        Spline c1 = Spline.getNullSpline();
        Spline c2 = Spline.getNullSpline();

        Field xpoly = c1.getClass().getDeclaredField("xpoly");
        xpoly.setAccessible(true);
        Field ypoly = c1.getClass().getDeclaredField("ypoly");
        ypoly.setAccessible(true);
        Field startPoint = c1.getClass().getDeclaredField("startPoint");
        startPoint.setAccessible(true);
        Field length = c1.getClass().getDeclaredField("length");
        length.setAccessible(true);

        xpoly.set(c1, c1x);
        ypoly.set(c1, c1y);
        startPoint.set(c1, new Point2d(c1x.apply(0), c1y.apply(0)));

        xpoly.set(c2, c2x);
        ypoly.set(c2, c2y);
        startPoint.set(c2, new Point2d(c2x.apply(0), c2y.apply(0)));

        length.set(c1, c1.displacementAt(1));
        length.set(c2, c2.displacementAt(1));

        road.addSpline(c1);
        road.addSpline(c2);
    }

    @Test
    void checkCurveLen(){
        try {
            buildTestRoad();
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.println(road.getSegmentAtDisplacement(0).getFirst().displacementAt(1));

        Pair<Spline, Double> output = road.getSegmentAtDisplacement(2);
        System.out.println(Arrays.toString(output.getFirst().getXCoeffs()));
        System.out.println(Arrays.toString(output.getFirst().getYCoeffs()));
        System.out.println(output.getSecond());

        Point2d point = road.pointAtDisplacement(1);
        System.out.printf("(%f, %f)%n", point.getX(), point.getY());
    }
}

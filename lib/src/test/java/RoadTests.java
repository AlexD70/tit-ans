import org.junit.jupiter.api.Test;
import titans.algebra.NPoly;
import titans.geometry.Point2d;
import titans.roads.Road;
import titans.roads.Spline;

import java.lang.reflect.Field;
import java.util.Arrays;

import org.apache.commons.math3.util.Pair;

import static org.junit.jupiter.api.Assertions.*;

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
    public Road road = new Road();

    public void buildTestRoad() {
        Spline c1 = Spline.getNullSpline();
        Spline c2 = Spline.getNullSpline();

        assertDoesNotThrow(() -> c1.setSpline(c1x, c1y, new Point2d(c1x.apply(0), c1y.apply(0))));
        assertDoesNotThrow(() -> c2.setSpline(c2x, c2y, new Point2d(c2x.apply(0), c2y.apply(0))));

        road.addSpline(c1, false);
        road.addSpline(c2, false);
    }

    @Test
    void checkCurveLen(){
        buildTestRoad();

        System.out.println(road.getSegmentAtDisplacement(0).getFirst().displacementAt(1));

        Pair<Spline, Double> output = road.getSegmentAtDisplacement(2);
        System.out.println(Arrays.toString(output.getFirst().getXCoeffs()));
        System.out.println(Arrays.toString(output.getFirst().getYCoeffs()));
        System.out.println(output.getSecond());

        Point2d point = road.pointAtDisplacement(1);
        System.out.printf("(%f, %f)%n", point.getX(), point.getY());
    }
}

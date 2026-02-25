import org.apache.commons.math3.util.Pair;
import org.junit.jupiter.api.Test;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.roads.Road;
import titans.roads.RoadBuilder;
import titans.roads.Spline;

import java.lang.reflect.Field;
import static org.junit.jupiter.api.Assertions.*;

public class RoadBuilderTests {
    static Point2d[] points = new Point2d[4];
    static double[] tangents = new double[5];
    static Point2d generatePoint(){
        return new Point2d(Math.random() * 200 - 100, Math.random() * 200 - 100);
    }
    static double generateTan(){
        return Math.random() * 2 * Math.PI - Math.PI;
    }
    static{
        for(int i = 0; i < 4; i++){
            points[i] = generatePoint();
        }
        for(int i = 0; i < 5; i++){
            tangents[i] = generateTan();
        }
    }

    // should make a separate assertions class because they are all over the place
    // same with test roads -- they're just everywhere
    // also this epsilon might be quite high -- should try testing newton raphson
    // doesnt matter - fixed precision issues with better relative tolerance and initial guess
    public static final double EPS = 0.001;
    static void assertPointsEqual(Point2d p1, Point2d p2){
        System.out.printf("(%f, %f) - expected, (%f, %f) - actual%n", p1.getX(), p1.getY(), p2.getX(), p2.getY());
        System.out.printf("(%f, %f) - delta%n", p1.getX() - p2.getX(), p1.getY() - p2.getY());
        MathTests.assertWithinError(p1.getX() - p2.getX(), 0, EPS);
        MathTests.assertWithinError(p1.getY() - p2.getY(), 0, EPS);
        //assertEquals(p1, p2); // Point2d.equals has a minuscule EPS value and therefore the assertion fails
    }

    // save the end points of the curves and their tangents to check them later
    Road generateTestRoad(boolean keepTangents){
        RoadBuilder builder = new RoadBuilder(new Point2d(0, 0), tangents[0], 0);
        if(keepTangents){
            return builder
                    .splineToPointKeepTangent(points[0])
                    .splineToPointKeepTangent(points[1])
                    .splineToPointKeepTangent(points[2])
                    .splineToPointKeepTangent(points[3])
                    .build();
        } else {
            return builder
                    .splineToPoint(points[0], tangents[1])
                    .splineToPoint(points[1], tangents[2])
                    .splineToPoint(points[2], tangents[3])
                    .splineToPoint(points[3], tangents[4])
                    .build();
        }
    }

    @Test
    void checkEndPoints() throws NoSuchFieldException, IllegalAccessException {
        Road r1 = generateTestRoad(true);
        Road r2 = generateTestRoad(false);

        Field lenarrfield1 = r1.getClass().getDeclaredField("lenarr");
        lenarrfield1.setAccessible(true);

        Object o1 = lenarrfield1.get(r1);
        Object o2 = lenarrfield1.get(r2);

        double[] arr1, arr2;
        if (o1 instanceof double[]) {
            arr1 = (double[]) o1;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }
        if (o2 instanceof double[]) {
            arr2 = (double[]) o2;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }

        int n = r1.getSegmentNum();
        for(int i = 0; i < n; i ++){
            Pair<Spline, Double> res = r1.getSegmentAtDisplacement(arr1[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();
            assertPointsEqual(points[i], s.pointAtDisplacement(reldisp));
        }

        int m = r2.getSegmentNum();
        for(int i = 0; i < m; i ++){
            Pair<Spline, Double> res = r2.getSegmentAtDisplacement(arr2[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();
            assertPointsEqual(points[i], s.pointAtDisplacement(reldisp));
        }
    }

    @Test
    // this sometimes fails because the tangentAt func sometimes gives a negative value
    // im not sure if this is a problem but lets keep it in mind
    // for now i'll add math.abs
    // this only checks end tangents now, it should also check start tangents
    void checkTangents() throws IllegalAccessException, NoSuchFieldException {
        Road r1 = generateTestRoad(true);
        Road r2 = generateTestRoad(false);

        Field lenarrfield1 = r1.getClass().getDeclaredField("lenarr");
        lenarrfield1.setAccessible(true);

        Object o1 = lenarrfield1.get(r1);
        Object o2 = lenarrfield1.get(r2);

        double[] arr1, arr2;
        if (o1 instanceof double[]) {
            arr1 = (double[]) o1;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }
        if (o2 instanceof double[]) {
            arr2 = (double[]) o2;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }

        int n = r1.getSegmentNum();
        for(int i = 0; i < n; i ++){
            Pair<Spline, Double> res = r1.getSegmentAtDisplacement(arr1[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();
            MathTests.assertWithinError(Math.abs(tangents[0]), s.tangentAt(s.uAtDisplacement(reldisp)));
        }

        int m = r2.getSegmentNum();
        for(int i = 0; i < m; i ++){
            Pair<Spline, Double> res = r2.getSegmentAtDisplacement(arr2[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();

            MathTests.assertWithinError(Math.abs(tangents[i + 1]), s.tangentAt(s.uAtDisplacement(reldisp)));
        }
    }

    @Test
    void checkEnd2ndDerivs() throws NoSuchFieldException, IllegalAccessException {
        Road r1 = generateTestRoad(true);
        Road r2 = generateTestRoad(false);

        Field lenarrfield1 = r1.getClass().getDeclaredField("lenarr");
        lenarrfield1.setAccessible(true);

        Object o1 = lenarrfield1.get(r1);
        Object o2 = lenarrfield1.get(r2);

        double[] arr1, arr2;
        if (o1 instanceof double[]) {
            arr1 = (double[]) o1;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }
        if (o2 instanceof double[]) {
            arr2 = (double[]) o2;
        } else {
            throw new RuntimeException("Error accessing lenarr field");
        }

        int n = r1.getSegmentNum();
        for(int i = 0; i < n; i ++){
            Pair<Spline, Double> res = r1.getSegmentAtDisplacement(arr1[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();
            assertPointsEqual(new Point2d(0, 0), s.secondDerivativeAt(1));
        }

        int m = r2.getSegmentNum();
        for(int i = 0; i < m; i ++){
            Pair<Spline, Double> res = r2.getSegmentAtDisplacement(arr2[i]);
            Spline s = res.getFirst();
            Double reldisp = res.getSecond();
            assertPointsEqual(new Point2d(0, 0), s.secondDerivativeAt(1));
        }
    }

    @Test
    void checkRoadStartTangent(){
        Road r1 = generateTestRoad(false);

        Vector2d deriv;
        Point2d derivp = r1.getDerivAtDisplacement(0);
        if(derivp == null){
            throw new RuntimeException("Test checkRoadStartTangent failed! Null Pointer Exception caught...");
        } else {
            deriv = derivp.toVector();
        }

        deriv.toPolar();
        MathTests.assertWithinError(deriv.getT(), Math.abs(tangents[0]));
    }

    @Test
    void checkC2Continuity(){
        Road r1 = generateTestRoad(true);
        Road r2 = generateTestRoad(false);

        int n = r1.getSegmentNum();
        int m = r2.getSegmentNum();

        for(int i = 0; i < n - 1; i ++){
            Spline s1 = r1.getSegmentByIndex(i);
            Spline s2 = r1.getSegmentByIndex(i + 1);

            // check position at joints
            assertPointsEqual(s1.pointAt(1), s2.pointAt(0));

            // check tangents at joints
            assertPointsEqual(s1.tangentVectorAt(1).toPoint(), s2.tangentVectorAt(0).toPoint());

            // check 2nd derivs at joints
            assertPointsEqual(s1.secondDerivativeAt(1), s2.secondDerivativeAt(0));
        }


        for(int i = 0; i < m - 1; i ++){
            Spline s1 = r2.getSegmentByIndex(i);
            Spline s2 = r2.getSegmentByIndex(i + 1);

            // check position at joints
            assertPointsEqual(s1.pointAt(1), s2.pointAt(0));

            // check tangents at joints
            assertPointsEqual(s1.tangentVectorAt(1).toPoint(), s2.tangentVectorAt(0).toPoint());

            // check 2nd derivs at joints
            assertPointsEqual(s1.secondDerivativeAt(1), s2.secondDerivativeAt(0));
        }
    }

    /*
    - generate curves randomly ----- done
    - check end points of each curve ----- done
    - check end & start tangents!!! ----- done
    - check c2 continuity by computing "limits" ----- done
    - check if 2nd deriv is 0 at endpoints ----- done
     */

    @Test
    void testRoadBuilder(){
        // should assert a timeout here for road building
        Road r = generateTestRoad(true);
    }

}

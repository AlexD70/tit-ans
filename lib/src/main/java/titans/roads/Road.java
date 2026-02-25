package titans.roads;

import org.apache.commons.math3.util.Pair;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.util.*;

import org.javatuples.Triplet;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.*;

public class Road {
    private final static int MAX_SEGMENTS = 30;

    protected ArrayList<Spline> segments = new ArrayList<>(MAX_SEGMENTS + 1);
    protected ArrayList<Boolean> isContinous = new ArrayList<>(MAX_SEGMENTS + 1);
    protected double[] lenarr = new double[MAX_SEGMENTS + 1];

    private double totalLength = 0;
    private int n = 0;
    private double eps = 0.005;

    public Road(){}

    public void addSpline(Spline s, boolean continuous){
        if(n == MAX_SEGMENTS){
            throw new RuntimeException("Max segment count exceeded for path!");
        }

        segments.add(s);
        isContinous.add(continuous);
        totalLength += s.length;
        lenarr[n] = totalLength;
        n += 1;
    }

    private int binSearchRecursive(int start, int end, double search){
        if(search <= lenarr[start]){
            return start;
        }

        if(start == end - 1){
            return end;
        }

        if(lenarr[(start + end) / 2] >= search){
            return binSearchRecursive(start, (start + end) / 2, search);
        } else {
            return binSearchRecursive((start + end) / 2, end, search);
        }
    }

    // TODO: write a test for this one
    public Pair<Spline, Double> getSegmentAtDisplacement(double d){
        if(d < 0 || d > totalLength){
            throw new RuntimeException("Displacement out of bounds!");
        }

        int index = binSearchRecursive(0, n, d);
        return new Pair<>(segments.get(index), Math.abs(d - ((index == 0)?(0):(lenarr[index - 1]))));
    }

    public Triplet<Spline, Double, Boolean> getSgmIndexAtDisp(double d){
        if(d < 0 || d > totalLength){
            throw new RuntimeException("Displacement out of bounds!");
        }

        int index = binSearchRecursive(0, n, d);
        double delta_d = Math.abs(d - ((index == 0)?(0):(lenarr[index - 1])));
        return new Triplet<>(segments.get(index), delta_d, (delta_d < eps)?(isContinous.get(index)):(true));
    }

    public int getSegmentIndexAtDisplacement(double d){
        if(d < 0 || d > totalLength){
            throw new RuntimeException("Displacement out of bounds!");
        }

        int index = binSearchRecursive(0, n, d);
        return index;
    }

    public Point2d pointAtDisplacement(double d){
        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
        return output.getFirst().pointAtDisplacement(output.getSecond());
    }


    @Useless
    public double slopeAtDisplacement(double d){
        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
        Spline s = output.getFirst();
        return s.tangentAt(s.uAtDisplacement(output.getSecond()));
    }

    public @Nullable Point2d getDerivAtDisplacement(double d){
        Triplet<Spline, Double, Boolean> segment = this.getSgmIndexAtDisp(d);
        Spline s = segment.getValue0();
        if(! segment.getValue2()){ // segment breaks continuity
            return null;
        }
        return s.firstDerivativeAt(s.uAtDisplacement(segment.getValue1()));
    }

    public @Nonnull Point2d getDerivAtDispNonnull(double d){
        Triplet<Spline, Double, Boolean> segment = this.getSgmIndexAtDisp(d);
        Spline s = segment.getValue0();
        return s.firstDerivativeAt(s.uAtDisplacement(segment.getValue1()));
    }

    public @Nullable Double getCurvatureAtDisplacement(double d){
        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
        Spline s = output.getFirst();
        System.out.print("output.second - u ");

        double u = s.uAtDisplacement(output.getSecond());
        System.out.println(u);
        Vector2d vec1stDeriv = s.firstDerivativeAt(u).toVector();
        Vector2d vec2ndDeriv = s.secondDerivativeAt(u).toVector();

        return vec1stDeriv.cross(vec2ndDeriv) / Math.pow(vec1stDeriv.abs(), 3);
    }

    public double getLength(){
        return totalLength;
    }

    public int getSegmentNum(){
        return n;
    }

    public Spline getSegmentByIndex(int idx){
        return segments.get(idx);
    }

//    public Point2d getDerivAtDisplacement(double d){
//        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
//        Spline s = output.getFirst();
//
//        Point2d deriv = s.firstDerivativeAt(output.getSecond());
//        double abs = deriv.toVector().abs();
//
//        return new Point2d(deriv.getX() / abs, deriv.getY() / abs);
//    }
}

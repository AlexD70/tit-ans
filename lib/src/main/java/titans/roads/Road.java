package titans.roads;

import org.apache.commons.math3.util.Pair;
import titans.geometry.Point2d;

import java.util.*;

public class Road {
    private final static int MAX_SEGMENTS = 30;

    protected ArrayList<Spline> segments = new ArrayList<>(MAX_SEGMENTS + 1);
    private double[] lenarr = new double[MAX_SEGMENTS + 1];

    private double totalLength = 0;
    private int n = 0;

    public Road(){}

    public void addSpline(Spline s){
        if(n == MAX_SEGMENTS){
            throw new RuntimeException("Max segment count exceeded for path!");
        }

        segments.add(s);
        totalLength += s.length;
        lenarr[n] = totalLength;
        n += 1;
    }

    private int binSearchRecursive(int start, int end, double search){
        if(search <= lenarr[0]){
            return 0;
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
        return new Pair<>(segments.get(index), d - lenarr[index - 1]);
    }

    public Point2d pointAtDisplacement(double d){
        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
        return output.getFirst().pointAtDisplacement(output.getSecond());
    }

    public double slopeAtDisplacement(double d){
        Pair<Spline, Double> output = getSegmentAtDisplacement(d);
        Spline s = output.getFirst();
        return s.tangentAt(s.uAtDisplacement(output.getSecond()));
    }
}

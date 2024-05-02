package titans.roads;

import titans.geometry.Point2d;

import java.util.*;

public class Road {
    protected Hashtable<Double, Spline> segments = new Hashtable<>(1);
    private double totalLength = 0;

    public Road(){}

    public void addSpline(Spline s){
        segments.put(s.length + totalLength, s);
        totalLength += s.length;
    }

    // TODO: write a unit test for this one
    public Spline getSegmentAtDisplacement(double d){
        boolean segmentFound = false;

        // this creates a reverse iterator for the keys of the hashtable
        ListIterator<Double> rtlIter = Collections.list(segments.keys()).listIterator(segments.size());

        double x = Collections.list(segments.keys()).get(segments.size());
        while(rtlIter.hasPrevious() && !segmentFound) {
            if(d <= x){
                segmentFound = true;
            } else {
                x = rtlIter.previous();
            }
        }

        if(!segmentFound){
            throw new RuntimeException("Segment not found!");
        }

        return segments.get(x);
    }

    public Point2d pointAtDisplacement(double d){
        return getSegmentAtDisplacement(d).pointAtDisplacement(d); // d - spline start
    }

    public double slopeAtDisplacement(double d){
        Spline s = getSegmentAtDisplacement(d);
        return s.tangentAt(s.uAtDisplacement(d)); // d - spline start
    }

}

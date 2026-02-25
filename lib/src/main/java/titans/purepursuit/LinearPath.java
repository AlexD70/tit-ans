package titans.purepursuit;

import titans.geometry.Line2d;
import titans.geometry.Point2d;

import java.util.ArrayList;

// this path is made for use with pure pursuit
// it uses a given number of waypoints and interpolates them linearly
// initial waypoint is either (0, 0) or user-defined
public class LinearPath {
    ArrayList<Point2d> waypoints = new ArrayList<>(5);
    ArrayList<Line2d> linesegments = new ArrayList<>(4);
    private int n = 0;
    private boolean isBuilt = false;

    public LinearPath(){
        addWaypoint(new Point2d(0, 0));
    }
    public LinearPath(Point2d startPoint){
        addWaypoint(startPoint);
    }

    public LinearPath addWaypoint(Point2d wayp){
        if(isBuilt){
            throw new RuntimeException("Cannot add waypoints after building path");
        }
        n += 1;
        waypoints.ensureCapacity(n);
        waypoints.add(wayp);

        return this;
    }

    public LinearPath build(){
        isBuilt = true;
        if(n < 2){
            throw new RuntimeException("Linear path cannot be generated with less than 2 waypoints!");
        }

        for(int i = 0; i < n - 2; i++){
            linesegments.ensureCapacity(i + 1);
            linesegments.add(new Line2d(waypoints.get(i), waypoints.get(i + 1)));
        }

        return this;
    }

}

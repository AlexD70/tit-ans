package titans.geometry;

import org.javatuples.Pair;
import titans.util.Useless;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

//!TODO: FIXME
// plenty of this class' functions crash or have undefined behaviour if it is not
// built using 2 points.
// fix: if built with slope, compute 2nd point automatically

// a line in a 2d space
// this is just a blob, with limited features
public class Line2d {
    private double slope;
    private Point2d a = new Point2d(), b = new Point2d(0, 1);

//    private double nTerm;

    public Line2d(){}
    public Line2d(Point2d a, Point2d b){
        if(a.equals(b)){
            throw new RuntimeException("Line cannot be generated from identical points.");
        }

        this.a = a;
        this.b = b;

        slope = Math.atan2(b.y - a.y, b.x - a.x);
//        nTerm = a.y - slope * a.x;
    }
    public Line2d(Point2d a, double m){
        slope = m;
        this.a = a;
//        nTerm = a.y - m * b.x;
    }

    public Point2d pointAtParam(double u){
        return new Point2d((a.x + u * (b.x - a.x)), (b.y + u * (b.y - a.y)));
    }

    public static double getSlope(Point2d a, Point2d b){
        return (new Line2d(a, b)).slope;
    }

    // solves the equation system:
    // x = xA + t(xB - xA)
    // y = yA + t(yB - yA)
    // (x - xC)^2 + (y - yC)^2 = R^2
    // if there are no intersection points, this returns null
    //private Pair<Double, Double> cachedT = null;
    //private Circle2d tCachedFor = null;
    @Nullable
    public Pair<Double, Double> intersect_Circle(Circle2d c){
        double dx = b.x - a.x;
        double dy = b.y - a.y;

        // AX^2 + BX + C = 0
        double freeTerm = Math.pow(a.x - c.getCX(), 2) + Math.pow(a.y - c.getCY(), 2) - Math.pow(c.getR(), 2); // C
        double bTerm = 2 * (dx * (a.x - c.getCX()) + dy * (a.y - c.getCY())); // B
        double aTerm = dx * dx + dy * dy; // A

        double discriminant = Math.pow(bTerm, 2) - 4 * aTerm * freeTerm;
        if(discriminant < 0){
            return null;
        } else if (discriminant == 0){
            double t = - bTerm / (2 * aTerm);
            //cachedT = new Pair<>(t, null);
            //tCachedFor = c;

            return new Pair<>(t, null);
        } else {
            double sqrtDisc = Math.sqrt(discriminant);
            double t1 = (-bTerm + sqrtDisc) / (2 * aTerm);
            double t2 = (-bTerm - sqrtDisc) / (2 * aTerm);
            //cachedT = new Pair<>(t1, t2);
            //tCachedFor = c;

            return new Pair<>(t1, t2);
        }
    }

    @Nullable
    // this does the same as previous, but also adds the constraint
    // that t in [0, 1]. If the circle has intersections but t is outside
    // the interval bounds for those, the function will return null
    public Pair<Double, Double> segmentIntersect_Circle(Circle2d c){
        Pair<Double, Double> res = intersect_Circle(c);
        if(res == null){
            return null;
        }

        Double t1 = res.getValue0();
        Double t2 = res.getValue1();
        if(t1 != null && 0 <= t1 && t1 <= 1){
            ;; // pass
        } else {
            t1 = null;
        }

        if(t2 != null && 0 <= t2 && t2 <= 1){
            ;; // pass
        } else {
            t2 = null;
        }

        if(t1 == null && t2 == null){
            return null;
        }

        return new Pair<>(t1, t2);
    }

    @Useless
    public double getSlope(){
        return slope;
    }
}

package titans.roads;

import titans.algebra.NPoly;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.util.Unimplemented;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class RoadBuilder {
    private Point2d prevPoint = null;
    private Point2d startPoint;
    private double startDeriv;
    private double start2ndDeriv;
    private Road road = new Road();
    private double eps  = 0.001;

    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv){
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
    }

    @SuppressWarnings("unused")
    public RoadBuilder goLeftRight(double howmuch){
        return goByDirections(howmuch, 0);
    }

    @SuppressWarnings("unused")
    public RoadBuilder goForwardBackward(double howmuch){
        return goByDirections(0, howmuch);
    }

    @SuppressWarnings("unused")
    public RoadBuilder goByDirections(double leftRight, double forwardBackward){
        return lineToPoint(new Point2d(prevPoint.getX() + leftRight, prevPoint.getY() + forwardBackward));
    }

    // the original heuristic uses 1/2 * lastDist for the last spline, however the usage
    // we seek doesn't quite allow that, so let's leave it at max(dist1, dist2)
    @Nonnull private Vector2d tangentVector(double startTangent, @Nonnull Point2d start, @Nonnull Point2d end, @Nullable Point2d prev) {
        double r = 1;

        if(prev == null) {
            r = Point2d.dist(start, end);
        } else {
            r = 2 * Math.max(Point2d.dist(start, end), Point2d.dist(prev, start));
        }

        return Vector2d.fromPolar(r, startTangent);
    }

    // line parameterization
    @SuppressWarnings("unused")
    public RoadBuilder lineToPoint(Point2d endPoint){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        double[] xcoeffs = new double[] {0, 0, 0, 0, v.getX(), startPoint.getX()};
        double[] ycoeffs = new double[] {0, 0, 0, 0, v.getY(), startPoint.getY()};

        Spline s = new Spline();
        NPoly xpoly = new NPoly(5), ypoly =  new NPoly(5);
        xpoly.assignCoefficients(xcoeffs);
        ypoly.assignCoefficients(ycoeffs);
        s.xpoly = xpoly;
        s.ypoly = ypoly;
        s.length = v.abs();

        boolean cont = Math.abs(startDeriv - Line2d.getSlope(startPoint, endPoint)) < eps;
        road.addSpline(s, cont);

        startDeriv = Line2d.getSlope(startPoint, endPoint);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder splineToPointKeepTangent(Point2d endPoint){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();

        Spline s = Spline.buildSpline6(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }

    @SuppressWarnings("unused")
    public RoadBuilder splineToPoint(Point2d endPoint, double endTangent){
        Point2d startTangentPoint = tangentVector(startDeriv, startPoint, endPoint, prevPoint).toPoint();
        Point2d endTangentPoint = tangentVector(endTangent, startPoint, endPoint, prevPoint).toPoint();

        Spline s = Spline.buildSpline6(startPoint, endPoint, startTangentPoint, endTangentPoint, new Point2d(start2ndDeriv, start2ndDeriv), new Point2d(start2ndDeriv, start2ndDeriv));
        startDeriv = endTangent;
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s, true);

        return this;
    }

    // ???
    @SuppressWarnings("unused")
    @Unimplemented
    public RoadBuilder splineToPointAlter2ndDeriv(Point2d endPoint, double deriv2){
        return this;
    }

    @SuppressWarnings("unused")
    public Road build(){
        return road;
    }
}

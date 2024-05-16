package titans.roads;

import titans.algebra.NPoly;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class RoadBuilder {
    private Point2d prevPoint = null;
    private Point2d startPoint;
    private double startDeriv;
    private double start2ndDeriv;
    private Road road = new Road();

    public RoadBuilder(Point2d startPoint, double startDeriv, double start2ndDeriv){
        this.startPoint = startPoint;
        this.startDeriv = startDeriv;
        this.start2ndDeriv = start2ndDeriv;
    }

    public RoadBuilder goLeftRight(double howmuch){
        return goByDirections(howmuch, 0);
    }

    public RoadBuilder goForwardBackward(double howmuch){
        return goByDirections(0, howmuch);
    }

    public RoadBuilder goByDirections(double leftRight, double forwardBackward){
        return lineToPoint(new Point2d(startPoint.getX() + leftRight, startPoint.getY() + forwardBackward));
    }

    // the original heuristic uses 1/2 * lastDist for the last spline, however the usage
    // we seek doesn't quite allow that, so let's leave it at min(dist1, dist2)
    @Nonnull private Vector2d tangentVector(double startTangent, @Nonnull Point2d start, @Nonnull Point2d end, @Nullable Point2d prev) {
        double r = 1;

        if(prev == null) {
            r = Point2d.dist(start, end) / 2;
        } else {
            r = Math.min(Point2d.dist(start, end), Point2d.dist(prev, start));
        }

        return Vector2d.fromPolar(r, startTangent);
    }

    // line parameterization
    public RoadBuilder lineToPoint(Point2d endPoint){
        Vector2d v = endPoint.toVector().diff(startPoint.toVector());
        double[] xcoeffs = new double[] {0, 0, 0, v.getX(), startPoint.getX()};
        double[] ycoeffs = new double[] {0, 0, 0, v.getY(), startPoint.getY()};

        Spline s = new Spline();
        NPoly xpoly = new NPoly(5), ypoly =  new NPoly(5);
        xpoly.assignCoefficients(xcoeffs);
        ypoly.assignCoefficients(ycoeffs);
        s.xpoly = xpoly;
        s.ypoly = ypoly;

        road.addSpline(s);
        startDeriv = Line2d.getSlope(endPoint, startPoint);
        start2ndDeriv = 0;
        prevPoint = startPoint;
        startPoint = endPoint;

        return this;
    }

    public RoadBuilder splineToPointKeepTangent(Point2d endPoint){
        Vector2d endTangentVector = tangentVector(startDeriv, startPoint, endPoint, prevPoint);
        Point2d tangentVectorEndPoint = endTangentVector.toPoint();

        Spline s = Spline.buildSpline6(startPoint, endPoint, tangentVectorEndPoint, tangentVectorEndPoint, new Point2d(), new Point2d());
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s);

        return this;
    }

    public RoadBuilder splineToPoint(Point2d endPoint, double endTangent){
        Point2d startTangentPoint = tangentVector(startDeriv, startPoint, endPoint, prevPoint).toPoint();
        Point2d endTangentPoint = tangentVector(endTangent, startPoint, endPoint, prevPoint).toPoint();

        Spline s = Spline.buildSpline6(startPoint, endPoint, startTangentPoint, endTangentPoint, new Point2d(), new Point2d());
        startDeriv = endTangent;
        prevPoint = startPoint;
        startPoint = endPoint;
        road.addSpline(s);

        return this;
    }

    public RoadBuilder splineToPointAlter2ndDeriv(Point2d endPoint, double deriv2){
        return this;
    }

    public Road build(){
        return road;
    }
}

package titans.roads;

import titans.algebra.NPoly;
import titans.geometry.Line2d;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

public class RoadBuilder {
    private Point2d startPoint;
    private Point2d startDeriv;
    private Point2d start2ndDeriv;
    private Road road = new Road();

    public RoadBuilder(Point2d startPoint, Point2d startDeriv, Point2d start2ndDeriv){
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
        startPoint = endPoint;
        // ! do sth with the derivative

        return this;
    }

    public RoadBuilder splineToPointKeepTangent(Point2d endPoint){
        Spline s = Spline.buildSpline6(startPoint, endPoint, startDeriv, startDeriv, start2ndDeriv, start2ndDeriv);
        startPoint = endPoint;
        road.addSpline(s);

        return this;
    }

    public RoadBuilder splineToPoint(Point2d endPoint, double endTangent){
        Spline s = Spline.buildSpline6(startPoint, endPoint, startDeriv, startDeriv, start2ndDeriv, start2ndDeriv);
        startPoint = endPoint;
        road.addSpline(s);

        return this;
    }
}

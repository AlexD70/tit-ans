package titans.roads;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import titans.algebra.NPoly;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

public class Spline {
    private NPoly xpoly, ypoly;
    private Point2d startPoint, endPoint;

    public Spline(){
        xpoly = new NPoly(5);
        ypoly = new NPoly(5);
    }

    public Point2d pointAt(double u){
        return new Point2d(xpoly.apply(u), ypoly.apply(u));
    }

    // integrates the position vector over (0, u) to get the distance travelled
    public double displacementAt(double u){
        SimpsonIntegrator integrator = new SimpsonIntegrator();

        return integrator.integrate(
                10,
                (t) -> Math.sqrt(Math.pow(xpoly.apply(t) - startPoint.getX(), 2) + Math.pow(ypoly.apply(t) - startPoint.getY(), 2)),
                0,
                Integer.MAX_VALUE
        );
    }

    public double uAtDisplacement(double d){
         // skip checking if d > segment length for now

        UnivariateFunction fx = new UnivariateFunction() {
            @Override
            public double value(double x) {
                return displacementAt(x);
            }
        };

        BrentOptimizer brentopt = new BrentOptimizer(5e-8, 1e-10);
        UnivariatePointValuePair res = brentopt.optimize(
                new MaxIter(100), new UnivariateObjectiveFunction(fx),
                GoalType.MINIMIZE, new SearchInterval(0, 1)
        );

        return res.getPoint();
    }

    public Point2d pointAtDisplacement(double d){
        return pointAt(uAtDisplacement(d));
    }

    public Point2d firstDerivativeAt(double u) {
        return new Point2d(xpoly.getDerivative().apply(u), ypoly.getDerivative().apply(u));
    }

    public Point2d secondDerivativeAt(double u){
        return new Point2d(
                xpoly.getSecondDerivative().apply(u),
                ypoly.getSecondDerivative().apply(u)
        );
    }

    // tangent or slope of the curve
    public double tangentAt(double u){
        Point2d deriv = firstDerivativeAt(u);
        return deriv.getY() / deriv.getX();
    }

    // unimplemented
    // this aims to replace the unit_arc_length function in segment.py
    // the Vector2d class also needs a function to transform its x, y representation
    // into a polar one r(cos t + i * sin t)
    public Vector2d positionVectorAt(double u){
        return new Vector2d();
    }

    // unimplemented
    public static Spline buildSpline(
            Point2d start, Point2d end,
            Point2d startDeriv, Point2d endDeriv,
            Point2d start2ndDeriv, Point2d end2ndDeriv
    ){
        return null;
        // this results in an equation system with 6 variables and 6 equations

        // how do we solve this programmatically???
    }
}

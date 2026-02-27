package titans.roads;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.linear.SolutionCallback;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.univariate.BrentOptimizer;
import org.apache.commons.math3.optim.univariate.SearchInterval;
import org.apache.commons.math3.optim.univariate.UnivariateObjectiveFunction;
import org.apache.commons.math3.optim.univariate.UnivariatePointValuePair;
import titans.algebra.NPoly;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;
import titans.util.NullSplineErr;
import titans.util.Useless;

import java.util.Arrays;

public class Spline {
    NPoly xpoly = null, ypoly = null;
    public double length = -1;
    private boolean isNull = true;
    private Point2d startPoint = null, endPoint = null;
    private static final double[][] systemMatrix = {
            {0,  0,  0, 0, 0, 1},
            {0,  0,  0, 0, 1, 0},
            {0,  0,  0, 2, 0, 0},
            {1,  1,  1, 1, 1, 1},
            {5,  4,  3, 2, 1, 0},
            {20, 12, 6, 2, 0, 0}
    };
    private static final double[][] invertedSystemMatrix = {
            {-6,  -3, -0.5,  6,  -3,  0.5},
            { 15,  8,  1.5, -15,  7, -1},
            {-10, -6, -1.5,  10, -4,  0.5},
            { 0,   0,  0.5,  0,   0,  0},
            { 0,   1,  0,    0,   0,  0},
            { 1,   0,  0,    0,   0,  0}
    };

    protected Spline(){}

    public Point2d pointAt(double u){
        if(isNull) {
            throw new NullSplineErr();
        }
        return new Point2d(xpoly.apply(u), ypoly.apply(u));
    }

    // integrates the position vector over (0, u) to get the distance travelled
    public double displacementAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        if(u == 0){
            return 0;
        }
        SimpsonIntegrator integrator = new SimpsonIntegrator();

        try {
            return integrator.integrate(
                    1000,
                    (t) -> Math.sqrt(Math.pow(xpoly.getDerivative().apply(t), 2) + Math.pow(ypoly.getDerivative().apply(t), 2)),
                    0,
                    u
            );
        } catch (TooManyEvaluationsException e) {
            try {
                return integrator.integrate(
                        1_000_000,
                        (t) -> Math.sqrt(Math.pow(xpoly.getDerivative().apply(t), 2) + Math.pow(ypoly.getDerivative().apply(t), 2)),
                        0,
                        u
                );
            } catch (TooManyEvaluationsException e2) {
                // TODO: this shouldnt crash the whole program - it might be better to throw a new error or log the error
                e.printStackTrace();
                System.exit(-1);
            }
        }

        return 0;
    }

    /*
    public double uAtDisplacementNewton(double d){
        if(isNull) {
            throw new NullSplineErr();
        }

        UnivariateDifferentiableFunction fx = new UnivariateDifferentiableFunction() {
            @Override
            public DerivativeStructure value(DerivativeStructure t) throws DimensionMismatchException {
                return ;
            }

            @Override
            public double value(double x) {
                return displacementAt(x) - d;
            }
        }
    }*/

    public double uAtDisplacement(double d){
        if(isNull){
            throw new NullSplineErr();
        }
         // skip checking if d > segment length for now

        UnivariateFunction fx = new UnivariateFunction() {
            @Override
            public double value(double x) {
                // do the absolute value because brentopt
                // doesnt search for fx = 0 but rather searches the function minimum
                // and if the function happens to be negative at any point
                // then the brentopt returns that rather than the desired point
                return Math.abs(displacementAt(x) - d);
            }
        };

        BrentOptimizer brentopt = new BrentOptimizer(1e-6, 1e-7);
        UnivariatePointValuePair res = brentopt.optimize(
                new MaxEval(Integer.MAX_VALUE),
                new MaxIter(1000),
                new UnivariateObjectiveFunction(fx),
                GoalType.MINIMIZE, new SearchInterval(0, 1),
                // not sure this is a good first guess - some benchmarking required
                new InitialGuess(new double[]{0, 0.5, 1})
        );

        return res.getPoint();
    }

    public Point2d pointAtDisplacement(double d){
        if(isNull){
            throw new NullSplineErr();
        }
        return pointAt(uAtDisplacement(d));
    }

    public Point2d firstDerivativeAt(double u) {
        if(isNull){
            throw new NullSplineErr();
        }
        return new Point2d(xpoly.getDerivative().apply(u), ypoly.getDerivative().apply(u));
    }

    public Point2d secondDerivativeAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        return new Point2d(
                xpoly.getSecondDerivative().apply(u),
                ypoly.getSecondDerivative().apply(u)
        );
    }

    // tangent or slope of the curve
    public double tangentAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }
        Vector2d deriv = firstDerivativeAt(u).toVector();
        deriv.toPolar();
        return deriv.getT();
    }

    public Vector2d tangentVectorAt(double u){
        if(isNull){
            throw new NullSplineErr();
        }

        return firstDerivativeAt(u).toVector().norm();
    }

    // this aims to replace the unit_arc_length function in segment.py
    // the Vector2d class also needs a function to transform its x, y representation
    // into a polar one r(cos t + i * sin t)
    @Useless
    public Vector2d positionVectorAt(double u){
        if(isNull) {
            throw new NullSplineErr();
        }
        return new Vector2d(xpoly.apply(u), ypoly.apply(u));
    }

    public static Spline buildSpline6(
            Point2d start, Point2d end,
            Point2d startDeriv, Point2d endDeriv,
            Point2d start2ndDeriv, Point2d end2ndDeriv
    ){
        double[][] rValueX = {
                {start.getX()}, {startDeriv.getX()}, {start2ndDeriv.getX()},
                {end.getX()}, {endDeriv.getX()}, {end2ndDeriv.getX()}
        };
        double[][] rValueY = {
                {start.getY()}, {startDeriv.getY()}, {start2ndDeriv.getY()},
                {end.getY()}, {endDeriv.getY()}, {end2ndDeriv.getY()}
        };

        RealMatrix matrix = new BlockRealMatrix(invertedSystemMatrix);
        RealMatrix resultX = matrix.multiply(new BlockRealMatrix(rValueX));
        RealMatrix resultY = matrix.multiply(new BlockRealMatrix(rValueY));
        //System.out.println(resultX.toString());
        //System.out.println(resultY.toString());

        NPoly xpoly = new NPoly(5);
        xpoly.assignCoefficients(resultX.getColumn(0));
        NPoly ypoly = new NPoly(5);
        ypoly.assignCoefficients(resultY.getColumn(0));

        Spline spline = new Spline();
        spline.startPoint = start;
        spline.endPoint = end;
        spline.xpoly = xpoly;
        spline.ypoly = ypoly;
        spline.isNull = false;
        spline.length = spline.displacementAt(1);

        return spline;
    }

    // TODO: use LU decomp to solve here
    static RealMatrix A = MatrixUtils.createRealMatrix(systemMatrix);
    static LUDecomposition lu = new LUDecomposition(A);
    static DecompositionSolver solver = lu.getSolver();
    public static Spline buildSpline6_LU(
            Point2d start, Point2d end,
            Point2d startDeriv, Point2d endDeriv,
            Point2d start2ndDeriv, Point2d end2ndDeriv
    ){
        double[] rValueX = {
                start.getX(), startDeriv.getX(), start2ndDeriv.getX(),
                end.getX(), endDeriv.getX(), end2ndDeriv.getX()
        };
        double[] rValueY = {
                start.getY(), startDeriv.getY(), start2ndDeriv.getY(),
                end.getY(), endDeriv.getY(), end2ndDeriv.getY()
        };

        RealVector xvec = new ArrayRealVector(rValueX, false);
        RealVector yvec = new ArrayRealVector(rValueY, false);
        RealVector resultX = solver.solve(xvec);
        RealVector resultY = solver.solve(yvec);
        NPoly xpoly = new NPoly(5);
        xpoly.assignCoefficients(resultX.toArray());
        NPoly ypoly = new NPoly(5);
        ypoly.assignCoefficients(resultY.toArray());

        Spline spline = new Spline();
        spline.startPoint = start;
        spline.endPoint = end;
        spline.xpoly = xpoly;
        spline.ypoly = ypoly;
        spline.isNull = false;
        spline.length = spline.displacementAt(1);

        return spline;
    }

    public static Spline getNullSpline(){
        return new Spline();
    }

    public Double[] getXCoeffs(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return xpoly.getCoeffs();
    }

    public Double[] getYCoeffs(){
        if (isNull) {
            throw new NullSplineErr();
        }
        return ypoly.getCoeffs();
    }

    public void unsetNull() {
        if(!isNull){
            return;
        }
        if(xpoly != null && ypoly != null){
            isNull = false;
            this.length = displacementAt(1);
        } else {
            throw new RuntimeException("Cannot unset null with empty polynomials");
        }
    }

    public void setSpline(NPoly xpoly, NPoly ypoly, Point2d startPoint) {
        if(!isNull){
            throw new RuntimeException("Do not modify non-null splines!");
        }

        this.xpoly = xpoly;
        this.ypoly = ypoly;
        this.startPoint = startPoint;
        unsetNull();
    }
}

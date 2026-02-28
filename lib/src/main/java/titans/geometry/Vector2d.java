package titans.geometry;

import org.apache.commons.math3.util.FastMath;

// stub
// this represents a position vector
// v = xi + yj
// v = r(cos t + i sin t)
public class Vector2d {
    private double x, y;
    private double r, t;

    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2d fromPolar(double r, double t){
        Vector2d ret = new Vector2d(FastMath.cos(t) * r, FastMath.sin(t) * r);
        ret.r = r;
        ret.t = t;

        return ret;
    }

    // is this even right?
    public void toPolar(){
        r = FastMath.sqrt(FastMath.pow(x, 2) + FastMath.pow(y, 2));
        t = FastMath.acos(x/r);
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getT(){
        return t;
    }

    public double getR(){
        return r;
    }

    public double abs(){
        return FastMath.sqrt(FastMath.pow(x, 2) + FastMath.pow(y, 2));
    }

    /*
    cross(u1, u2) = | i   j   k | = (x1y2 - y1x2)k (for 2d vectors)
                    | x1  y1  0 |
                    | x2  y2  0 |
     */
    public double cross(Vector2d other){
        return this.x * other.y - this.y * other.x;
    }

    public Point2d toPoint(){
        return new Point2d(x, y);
    }

    public Vector2d mlt(double scalar){
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }

    @Override
    public boolean equals(Object obj){
        if (obj instanceof Vector2d vect) {
            return (FastMath.abs(vect.getX() - this.x) < 0.0001) && (FastMath.abs(vect.getY() - this.y) < 0.0001);
        } else {
            return false;
        }
    }

    public Vector2d norm() {
        return this.mlt(1 / this.abs());
    }

    public Vector2d diff(Vector2d other){
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d add(Vector2d other){
        return new Vector2d(this.x + other.x, this.y + other.y);
    }
}

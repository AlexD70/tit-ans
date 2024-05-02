package titans.geometry;

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
        Vector2d ret = new Vector2d(Math.cos(t) * r, Math.sin(t) * r);
        ret.r = r;
        ret.t = t;

        return ret;
    }

    // is this even right?
    public void toPolar(){
        r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        t = Math.acos(x/r);
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
}

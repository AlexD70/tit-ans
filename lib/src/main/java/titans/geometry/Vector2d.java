package titans.geometry;

// stub
// this represents a position vector
// v = xi + yj
// v = r(cos t + i sin t)
public class Vector2d {
    private double x, y;
    private double r, t;

    public Vector2d(){
        throw new RuntimeException("stub!");
    }

    // is this even right?
    public void toPolar(){
        r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        t = Math.acos(x/r);
    }
}

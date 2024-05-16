package titans.geometry;

// a point in a 2d space
// the constructor with no parameters returns the origin point
public class Point2d {
    double x = 0, y = 0;

    public Point2d(){}
    public Point2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof Point2d)){
            return false;
        }
        return (((Point2d)obj).x == x) && (((Point2d)obj).y == y);
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public Vector2d toVector(){
        return new Vector2d(x, y);
    }

    public static Vector2d distVector(Point2d a, Point2d b){
        return Vector2d.fromPolar(dist(a, b), Line2d.getSlope(a, b));
    }

    public static double dist(Point2d a, Point2d b){
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }
}

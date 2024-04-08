package titans.geometry;

// a point in a 2d space
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
}

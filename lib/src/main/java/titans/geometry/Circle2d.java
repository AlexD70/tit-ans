package titans.geometry;

// a circle in 2d space
// by default, the circle built is the unit circle
public class Circle2d {
    private Point2d center =  new Point2d();
    private double radius = 1;

    public Circle2d(){}
    public Circle2d(Point2d center, double radius){
        this.center = center;
        this.radius = radius;
    }
}

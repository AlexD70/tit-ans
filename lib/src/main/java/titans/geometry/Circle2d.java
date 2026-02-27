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

    // translate in the direction of the vector
    public Circle2d translate(Vector2d translation){
        this.center.translate(translation);
        return this;
    }

    public double getCX(){
        return center.x;
    }

    public double getCY(){
        return center.y;
    }

    public double getR(){
        return radius;
    }

    public boolean equals(Circle2d other){
        return other.center.equals(this.center) && (Math.abs(other.radius - this.radius) < 0.0001);
    }
}

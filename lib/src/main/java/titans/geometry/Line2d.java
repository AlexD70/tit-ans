package titans.geometry;

// a line in a 2d space
// this is just a blob, with limited features
public class Line2d {
    private double slope;
    private Point2d a = new Point2d(), b = new Point2d(0, 1);

//    private double nTerm;

    public Line2d(){}
    public Line2d(Point2d a, Point2d b){
        if(a.equals(b)){
            throw new RuntimeException("Line cannot be generated from identical points.");
        }

        this.a = a;
        this.b = b;

        slope = (b.y - a.y)/(b.x - a.x);
//        nTerm = a.y - slope * a.x;
    }
    public Line2d(Point2d a, double m){
        slope = m;
        this.a = a;
//        nTerm = a.y - m * b.x;
    }

    public static double getSlope(Point2d a, Point2d b){
        return (new Line2d(a, b)).slope;
    }

    public double getSlope(){
        return slope;
    }
}

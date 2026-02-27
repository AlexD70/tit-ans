package titans.roads;

import titans.geometry.Point2d;
import titans.geometry.Vector2d;

// unimplemented
public class KinematicStateXY {
    public Point2d position;
    public Vector2d velocity;
    public Vector2d acceleration;

    public KinematicStateXY(Point2d pos, Vector2d vel, Vector2d acc){
        position = pos;
        velocity = vel;
        acceleration = acc;
    }

    public static KinematicStateXY getLinearInterpolation(KinematicStateXY first, KinematicStateXY second, double timetotal, double timefromfirst){
        Vector2d possum = first.position.toVector().add(second.position.toVector());
        Vector2d velsum = first.velocity.add(second.velocity);
        Vector2d accsum = first.acceleration.add(second.acceleration);

        double interpolation_factor = timefromfirst / timetotal;

        possum.mlt(interpolation_factor);
        velsum.mlt(interpolation_factor);
        accsum.mlt(interpolation_factor);

        return new KinematicStateXY(possum.toPoint(), velsum, accsum);
    }
}

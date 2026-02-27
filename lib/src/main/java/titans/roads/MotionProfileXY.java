package titans.roads;

import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import java.util.ArrayList;

// stub
public class MotionProfileXY {
    Road road;
    double duration;

    // this describes the speed by displacement
    private ArrayList<Pair<Double, Double>> dispProfile = new ArrayList<>(100);
    public ArrayList<Pair<Double, KinematicStateXY>> timeProfile = new ArrayList<>();

    protected MotionProfileXY(){}

    // does not return endpoints
    private static double[] linearSpace(double start, double end, int res){
        double[] linspace = new double[res];
        double dist = end - start;

        double space = dist / (res + 1);
        for(int i = 0; i < res; i++){
            linspace[i] = (start = start + space);
        }

        return linspace;
    }

    // adds to time profile
    private void addToProfile(double time, Point2d xy, double vx, double vy, double ax, double ay){
        timeProfile.add(new Pair<>(time, new KinematicStateXY(
                xy, new Vector2d(vx, vy), new Vector2d(ax, ay)
        )));
    }

    // adds to a profile whose reference is passed on as an argument
    private void addToProfile(ArrayList<Pair<Double, KinematicState>> profile, double time, double x, double v, double a){
        profile.add(new Pair<>(time,  new KinematicState(x, v, a)));
    }

    public static MotionProfileXY buildProfile(Road road, ConstraintSetXY constraints, double initVel, double endVel, double initAcc, int resolution){
        double[] linspaceDisplacement = linearSpace(0, road.getLength(), resolution);

        double max_vel = constraints.maxVel;
        double max_ang_acc = constraints.maxAngAccel;
        double max_acc = constraints.maxAccel;

        MotionProfileXY profile = new MotionProfileXY();

        // add points between start and end on the displacement profile
        int i = 1;
        int sgm_num = 0, last_sgm = 0;
        for(double d : linspaceDisplacement){
            Double curvature = road.getCurvatureAtDisplacement(d);
            if(curvature == null){
                throw new RuntimeException("FATAL ERROR WHILE BUILDING MOTION PROFILE. ABORTING");
            }

            //System.out.print("curvature: "); System.out.println(curvature);
            Point2d tanPoint = road.getDerivAtDisplacement(d);
            double tan = 0;
            if(tanPoint == null){
                tan = Double.POSITIVE_INFINITY;
            } else {
                tan = tanPoint.toVector().abs();
            }

            double vmax_ang = constraints.maxAngVel / FastMath.abs(curvature * 2);
            double vmax2 = Double.POSITIVE_INFINITY;

            sgm_num = road.getSegmentIndexAtDisplacement(d);
            if(sgm_num != last_sgm){
                last_sgm = sgm_num;
                double waypoint_dist = road.lenarr[sgm_num] - road.lenarr[sgm_num - 1];
                double time = FastMath.sqrt(2 * waypoint_dist / max_acc);
                vmax2 = max_acc * time;

                if(Double.isInfinite(tan)){
                    vmax2 = 0;
                }
            }

            if (Double.isNaN(vmax_ang) || Double.isInfinite(vmax_ang) || curvature < 0.01){
                vmax_ang = max_vel + 1;
            }
            if(Double.isInfinite(vmax2)){
                vmax2 = max_vel + 1;
            }
            double plannedVel = FastMath.min(FastMath.min(max_vel, vmax_ang), vmax2);
            System.out.printf("%f planned vel%n", plannedVel);
            profile.dispProfile.add(new Pair<>(d, plannedVel));
            i++;
        }

        // add the end and the start to the displacement profile
        profile.dispProfile.set(0, new Pair<>(0d, initVel));
        profile.dispProfile.add(new Pair<>(road.getLength(), endVel));

        // first pass through the disp profile
        // this pass lowers the consecutive vel values
        // which require the robot to have an unattainable acceleration
        // to get to
        for (int j = 1; j < i; j ++){
            Pair<Double, Double> current = profile.dispProfile.get(j);
            Pair<Double, Double> prev = profile.dispProfile.get(j - 1);

            double currentVel = current.getSecond(), prevVel = prev.getSecond();
            if(currentVel > prevVel){
                double d = current.getFirst();
                double deltaDisp = d - prev.getFirst();

                double maxVel = FastMath.sqrt(FastMath.pow(prevVel, 2) + 2 * deltaDisp * max_acc);
                profile.dispProfile.set(j, new Pair<>(d, FastMath.min(maxVel, currentVel)));
            }
        }

        // backwards pass
        for (int j = i - 1; j > 0; j--){
            Pair<Double, Double> current = profile.dispProfile.get(j);
            Pair<Double, Double> next = profile.dispProfile.get(j - 1);

            double currentVel = current.getSecond(), nextVel = next.getSecond();
            if(currentVel < nextVel){
                double d1 = current.getFirst(), d2 = next.getFirst();
                double deltaDisp = d1 - d2;

                double maxVel = FastMath.sqrt(FastMath.pow(currentVel, 2) + 2 * deltaDisp * max_acc);
                profile.dispProfile.set(j - 1, new Pair<>(d2, FastMath.min(maxVel, nextVel)));
            }
        }

        // turn to time based profile
        profile.timeProfile.ensureCapacity(i);

        profile.addToProfile(
                0d,
                road.pointAtDisplacement(0),
                initVel,
                initVel,
                initAcc,
                initAcc
        );

        Vector2d tanVec = road.getDerivAtDispNonnull(0).toVector();
        tanVec.norm();

        double lastTime = 0, time = 0;
        double lastVel, currentVel;
        for (int j = 1; j < i; j ++){
            Pair<Double, Double> current = profile.dispProfile.get(j), last = profile.dispProfile.get(j - 1);

            currentVel = current.getSecond();
            lastVel = last.getSecond();
            double ds = current.getFirst() - last.getFirst();
            double dt = FastMath.abs(ds / ((currentVel + lastVel) / 2)); // ds / med_vel
            time += dt;

            Point2d tanPoint = road.getDerivAtDispNonnull(current.getFirst());
            tanVec.norm(); // normalize the vector

            double vx = currentVel * tanVec.getX(), vy = currentVel * tanVec.getY();
            double accel = (currentVel - lastVel) / (time - lastTime);
            double ax = accel * tanVec.getX(), ay = accel * tanVec.getY();

            Point2d point = road.pointAtDisplacement(current.getFirst());
            profile.addToProfile(
                    time,
                    point,
                    vx, vy, ax, ay
            );

            lastTime = time;
        }

        profile.duration = time;
        return profile;
    }
}

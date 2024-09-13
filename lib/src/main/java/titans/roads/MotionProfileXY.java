package titans.roads;

import org.apache.commons.math3.util.Pair;
import titans.geometry.Vector2d;

import java.util.ArrayList;

// stub
public class MotionProfileXY {
    Road road;

    // this describes the speed by displacement
    private ArrayList<Pair<Double, Double>> dispProfile = new ArrayList<>(100);
    public ArrayList<Pair<Double, KinematicStateXY>> timeProfile = new ArrayList<>();

    protected MotionProfileXY(){}

    // does not return endpoints
    private static double[] linearSpace(double start, double end, int res){
        double[] linspace = new double[res];
        double dist = end - start;

        double space = dist / (res + 1);
        for(int i = 0; i <= res; i++){
            linspace[i] = (start = start + space);
        }

        return linspace;
    }

    public static MotionProfileXY buildProfile(Road road, ConstraintSetXY constraints, double initVel, double endVel, double initAcc, int resolution){
        double[] linspaceDisplacement = linearSpace(0, road.getLength(), resolution);

        double max_vel = constraints.maxVel;
        double max_ang_acc = constraints.maxAngAccel;
        double max_acc = constraints.maxAccel;

        MotionProfileXY profile = new MotionProfileXY();

        // add points between start and end on the displacement profile
        int i = 1;
        for(double d : linspaceDisplacement){
            double curvature = road.getCurvatureAtDisplacement(d);
            double vmax_ang = max_ang_acc / Math.abs(curvature);

            double plannedVel = Math.min(max_vel, vmax_ang);
            profile.dispProfile.set(i, new Pair<>(d, plannedVel));
            i++;
        }

        // add the end and the start to the displacement profile
        profile.dispProfile.set(0, new Pair<>(0d, initVel));
        profile.dispProfile.set(i, new Pair<>(road.getLength(), endVel));

        // first pass through the disp profile
        // this pass lowers the consecutive vel values
        // which require the robot to have an unattainable acceleration
        // to get to
        for (int j = 1; j < i; j ++){
            Pair<Double, Double> current = profile.dispProfile.get(j);
            Pair<Double, Double> prev = profile.dispProfile.get(j - 1);

            double currentVel = current.getSecond();
            if(currentVel > prev.getSecond()){
                double d = current.getFirst();
                double deltaDisp = d - prev.getFirst();

                double maxVel = Math.sqrt(Math.pow(currentVel, 2) + 2 * deltaDisp * max_acc);
                profile.dispProfile.set(j, new Pair<>(d, Math.min(maxVel, currentVel)));
            }
        }

        // backwards pass
        for (int j = i - 1; j > 0; j--){
            Pair<Double, Double> current = profile.dispProfile.get(j);
            Pair<Double, Double> next = profile.dispProfile.get(j - 1);

            double currentVel = current.getSecond();
            if(currentVel < next.getSecond()){
                double d1 = current.getFirst(), d2 = next.getFirst();
                double deltaDisp = d1 - d2;
                double nextVel = next.getSecond();

                double maxVel = Math.sqrt(Math.pow(currentVel, 2) + 2 * deltaDisp * max_acc);
                profile.dispProfile.set(j - 1, new Pair<>(d2, Math.min(maxVel, nextVel)));
            }
        }

        // turn to time based profile
        profile.timeProfile.ensureCapacity(i);
        profile.timeProfile.add(new Pair<>(0d, new KinematicStateXY(
                road.pointAtDisplacement(0),
                new Vector2d(initVel, initVel),
                new Vector2d(initAcc, initAcc)
        )));
        //TODO


        return profile;
    }
}

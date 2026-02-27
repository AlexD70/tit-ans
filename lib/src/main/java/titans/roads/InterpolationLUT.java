package titans.roads;

import org.apache.commons.math3.util.Pair;
import titans.geometry.Point2d;
import titans.geometry.Vector2d;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.function.Function;

public class InterpolationLUT {
    public ArrayList<Double> keys;
    public ArrayList<KinematicStateXY> values;

    public int first = 0, second = 1;
    public double eps = 0.001;
    public int maxIter = 3;

    public KinematicStateXY getStateAt(double t){
        double time1 = keys.get(first);
        double time2 = keys.get(second);

        int iter = 0;
        while(iter < maxIter) {
            if (Math.abs(t - first) <= eps) {
                // do not go to next here
                return values.get(first);

            } else if (Math.abs(t - second) <= eps) {
                first += 1;
                second += 1;
                return values.get(second);

            } else if (time1 < t && t < time2) {
                // do not go to next in this branch
                KinematicStateXY state1 = values.get(first);
                KinematicStateXY state2 = values.get(second);
                double totaltime = Math.abs(time2 - time1);
                double dtime = Math.abs(t - time1);

                return KinematicStateXY.getLinearInterpolation(state1, state2, totaltime, dtime);

            } else {
                first += 1;
                second += 1;
                time1 = keys.get(first);
                time2 = keys.get(second);
            }
        }

        // throw error if the correct value cannot be found in 4 iterations
        throw new RuntimeException("Interpolation LUT crashed at runtime. Iteration number exceeds maxIter.");
    }

    // should not be used by the user
    // only for internal library use
    public void forceReset(){
        first = 0;
        second = 1;
    }

    // only for internal library use
    public void setMaxIter(int newMaxIter){
        this.maxIter = newMaxIter;
    }

    public InterpolationLUT(ArrayList<Double> keys, ArrayList<KinematicStateXY> values){
        this.keys = keys;
        this.values = values;
        //this.maxIter = this.values.size() + 1;
    }

    public static InterpolationLUT linear_fromMotionProfile(MotionProfileXY profile){
        ArrayList<Double> keys = new ArrayList<>(profile.timeProfile.size());
        ArrayList<KinematicStateXY> kinematicstates = new ArrayList<>(profile.timeProfile.size());
        for (Pair<Double, KinematicStateXY> p : profile.timeProfile){
            keys.add(p.getFirst());
            kinematicstates.add(p.getSecond());
        }

        return new InterpolationLUT(keys, kinematicstates);
    }

}

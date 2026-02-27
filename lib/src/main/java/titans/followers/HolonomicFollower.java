package titans.followers;

import titans.geometry.Vector2d;
import titans.roads.*;
import titans.util.Feedforward;
import titans.util.PID;

@SuppressWarnings("unused")
public class HolonomicFollower {
    PID xPID, yPID;
    Feedforward feedforward;
    InterpolationLUT lut;
    Road road;
    double lateralMlt = Math.sqrt(2);

    public HolonomicFollower(double kV, double kA, double kStatic, double kP, double kD, double kI, double lateralMultiplier){
        feedforward = new Feedforward(kV, kA, kStatic);
        xPID = new PID(kP, kI, kD);
        yPID = new PID(kP, kI, kD);
        this.lateralMlt = lateralMultiplier;
    }

    public HolonomicFollower(double kV, double kA, double kStatic, double kP, double kD, double kI){
        feedforward = new Feedforward(kV, kA, kStatic);
        xPID = new PID(kP, kI, kD);
        yPID = new PID(kP, kI, kD);
    }

    public void followRoad(Road road, ConstraintSetXY constraints, double initVel, double endVel, double initAcc){
        MotionProfileXY profile = MotionProfileXY.buildProfile(road, constraints, initVel, endVel, initAcc, 2000);
        lut = InterpolationLUT.linear_fromMotionProfile(profile);
        this.road = road;
        reset();
    }

    public void followRoad(Road road, ConstraintSetXY constraints){
        followRoad(road, constraints, 0, 0, 0);
    }

    public void reset(){
        xPID.resetIntegral();
        xPID.setTargetPosition(0);
        yPID.resetIntegral();
        yPID.setTargetPosition(0);
    }

    // calculate applies both the pids and the feedforward to the inputs and returns
    // a vector containing ux and uy
    // to get voltages for each wheel:
    // u1 = ux - uy
    // u2 = ux + uy
    // u3 = ux + uy
    // u4 = ux - uy
    // these formulas do not include turn as this project currently doesnt use
    // heading interpolation to be able to turn the robot
    public Vector2d calculate(Vector2d deltaPos, Vector2d vel, Vector2d accel){
        reset();
        double ux = xPID.update(deltaPos.getX()) + feedforward.update(vel.getX(), accel.getX());
        double uy = yPID.update(deltaPos.getY()) + feedforward.update(vel.getY(), accel.getY());
        return new Vector2d(ux, uy * lateralMlt);
    }

    public Vector2d calculate(double time, Vector2d currentPos){
        KinematicStateXY state = lut.getStateAt(time);
        Vector2d deltaPos = state.position.toVector().diff(currentPos);

        return calculate(deltaPos, state.velocity, state.acceleration);
    }
}

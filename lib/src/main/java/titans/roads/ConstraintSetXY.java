package titans.roads;

public class ConstraintSetXY {
    public double maxVel;
    public double maxAccel;
    public double maxAngVel;
    public double maxAngAccel;

    public ConstraintSetXY(double maxVel, double maxAccel, double maxAngVel, double maxAngAccel){
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxAngAccel = maxAngAccel;
        this.maxAngVel = maxAngVel;
    }
}

package titans.util;

@SuppressWarnings("unused")
public class Feedforward {
    public double kV = 0, kA = 0, kStatic = 0;

    public Feedforward(double kV, double kA, double kStatic){
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
    }

    public double update(double targetV, double targetA){
        double part = targetV * kV + targetA * kA;
        return part + Math.signum(part) * kStatic;
    }
}

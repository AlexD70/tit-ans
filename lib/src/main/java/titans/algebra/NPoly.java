package titans.algebra;

import java.util.ArrayList;

// nth order polynomial
public class NPoly {
    private ArrayList<Double> coeffs = new ArrayList<>(3);
    private int n = 2;

    public NPoly(){}

    public NPoly(int degree){
        coeffs.ensureCapacity(degree + 1);
        n = degree;
    }

    // coefficients are assigned left to right from the highest power of x to the lowest
    // coef[0] -> x^n
    // coef[1] -> x^(n - 1)
    // ...
    // coef[n] -> free term (x^0)
    // any excess coefficients are ignored
    // if less than n + 1 coefficients are provided, the rest of them are assumed to be 0
    public void assignCoefficients(double... coef){
        int i = 0;

        for(double c : coef){
            if(i > n){
                break;
            }
            coeffs.add(c);
            i ++;
        }

        for(; i <= n; i++){
            coeffs.add(0d);
        }
    }

    public double apply(double x){
        double xpowi = 1, res = 0;
        for(int i = 0; i <= n; i ++){
            res += coeffs.get(n - i) * xpowi;
            xpowi *= x;
        }

        return res;
    }

    public NPoly getDerivative(){
        NPoly deriv = new NPoly(n - 1);

        for(int i = 0; i < n; i ++){
            deriv.coeffs.add(coeffs.get(i) * (n - i));
        }

        return deriv;
    }

    public Double[] getCoeffs(){
        return coeffs.toArray(new Double[6]);
    }

    public NPoly getSecondDerivative(){
        return getDerivative().getDerivative();
    }
}

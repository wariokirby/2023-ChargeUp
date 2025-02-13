package frc.robot.Util;

public class PDController {
    double P_CONSTANT;
    double D_CONSTANT;
    
    double lastError = 0;

    public PDController(double p_CONSTANT, double d_CONSTANT) {
        P_CONSTANT = p_CONSTANT;
        D_CONSTANT = d_CONSTANT;
    }

    public double tick(double currentError) {
        double p_correct = P_CONSTANT * currentError;
        double d_correct = D_CONSTANT * (currentError - lastError);

        lastError = currentError;
        
        return p_correct + d_correct;
    }

    public void reset() {
        lastError = 0;
    }

    public PDController clone() {
        return new PDController(P_CONSTANT, D_CONSTANT);
    }
}

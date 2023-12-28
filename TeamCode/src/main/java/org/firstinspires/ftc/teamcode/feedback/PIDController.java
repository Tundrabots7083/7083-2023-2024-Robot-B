package org.firstinspires.ftc.teamcode.feedback;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    PIDCoefficients coefficients;

    protected boolean hasRun = false;

    protected ElapsedTime timer = new ElapsedTime();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    protected double minIntegralBound = -1;
    protected double maxIntegralBound = 1;

    public PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    /**
     * calculate PID output
     * @param reference the target position
     * @param state current system state
     * @return PID output
     */
    public double calculate(double reference, double state) {
        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error,dt);
        integrate(error,dt);
        previousError = error;
        return error * coefficients.Kp
                + integralSum * coefficients.Ki
                + derivative * coefficients.Kd;
    }

    public void setIntegrationBounds(double min, double max) {
        minIntegralBound = min;
        maxIntegralBound = max;
    }

    /**
     * get the time constant
     * @return time constant
     */
    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.time();
        timer.reset();
        return dt;
    }

    protected double calculateError(double reference, double state) {
        return reference - state;
    }

    protected void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
        integralSum = Range.clip(integralSum, minIntegralBound, maxIntegralBound);
    }

    protected double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;
        return derivative;
    }

}

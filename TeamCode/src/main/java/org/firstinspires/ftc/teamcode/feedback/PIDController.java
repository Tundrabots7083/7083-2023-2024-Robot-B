package org.firstinspires.ftc.teamcode.feedback;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

   private final double Kp;
   private final double Ki;
   private final double Kd;

    protected boolean hasRun = false;

    protected ElapsedTime timer = new ElapsedTime();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    protected double minIntegralBound = -1;
    protected double maxIntegralBound = 1;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * calculate PID output
     *
     * @param reference the target position
     * @param state     current system state
     * @return PID output
     */
    public double calculate(double reference, double state) {
        if (Double.isNaN(reference)) {
            return 0;
        }

        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;
        return error * Kp
                + integralSum * Ki
                + derivative * Kd;
    }

    public void setIntegrationBounds(double min, double max) {
        minIntegralBound = min;
        maxIntegralBound = max;
    }

    /**
     * get the time constant
     *
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

    public void reset() {
        hasRun = false;
        timer.reset();

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

        if (Double.isNaN(derivative)) {
            derivative = 0;
        }

        return derivative;
    }

}

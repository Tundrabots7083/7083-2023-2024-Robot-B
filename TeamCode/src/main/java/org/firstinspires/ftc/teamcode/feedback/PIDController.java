package org.firstinspires.ftc.teamcode.feedback;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * A PID Controller is a control loop mechanism employing feedback by continuously modulating
 * control. The PID Controller has three values that the programmer tunes. These values are
 * Kp, Ki, and Kd. These values are multiplied by their corresponding input. Changing these values
 * changes how the controller behaves.
 */
public class PIDController {

    public PIDCoefficients coefficients;

    protected boolean hasRun = false;

    protected ElapsedTime timer = new ElapsedTime();

    protected double previousError = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    protected double minIntegralBound = -1;
    protected double maxIntegralBound = 1;

    /**
     * Creates a new PID controller with the given PID coefficients.
     *
     * @param coefficients the PID coefificients to be used for the PID controller.
     */
    public PIDController(PIDCoefficients coefficients) {
        this.coefficients = coefficients;
    }

    /**
     * Calculate PID output
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
        return error * coefficients.Kp
                + integralSum * coefficients.Ki
                + derivative * coefficients.Kd;
    }

    /**
     * Set the upper and lower integral bounds.
     *
     * @param min the lower integral bound.
     * @param max the upper integral bound.
     */
    public void setIntegrationBounds(double min, double max) {
        minIntegralBound = min;
        maxIntegralBound = max;
    }

    /**
     * Get the time constant
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

    /**
     * Resets the PID controller values.
     */
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

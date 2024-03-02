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

    private final double Kp; // Proportional term, multiplied directly by the state error
    private final double Ki; // Integral term, multiplied directly by the state error integral
    private final double Kd; // Derivative term, multiplied directly by the state error rate of change
    private final ElapsedTime timer = new ElapsedTime();
    private double lastReference = 0;
    private boolean hasRun = false;
    private double previousError = 0;
    private double integralSum = 0;

    private double minIntegralBound = -1;
    private double maxIntegralBound = 1;

    /**
     * Creates a new PID controller with the given PID coefficients.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change.
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Calculate PID output
     *
     * @param reference the target position
     * @param state     current system state
     * @return PID output
     */
    public double calculate(double reference, double state) {
        // Make sure reference is a valid number
        if (Double.isNaN(reference)) {
            return 0;
        }

        // Calculate the PID values. If the target has changed, reset the integral sum
        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error, dt);
        integralSum = reference == lastReference ? integrate(error, dt) : 0;

        // Save the error and target
        previousError = error;
        lastReference = reference;

        return error * Kp
                + integralSum * Ki
                + derivative * Kd;
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

    /**
     * Returns the error between the target position and the current position.
     *
     * @param reference the target position.
     * @param state     the current position.
     * @return the error between the target position and the current position.
     */
    private double calculateError(double reference, double state) {
        return reference - state;
    }

    /**
     * Calculates the integral sum for the PID controller.
     *
     * @param error the error between the target position and the current position.
     * @param dt    the time since the PID controller output was last calculated.
     * @return the integral sum for the PID controller.
     */
    private double integrate(double error, double dt) {
        double integralSum = this.integralSum + error * dt;
        return Range.clip(integralSum, minIntegralBound, maxIntegralBound);
    }

    /**
     * Calculates the derivative for the PID controller.
     *
     * @param error the error between the target position and the current position.
     * @param dt    the time since the PID controller output was last calculated.
     * @return the derivative for the PID controller.
     */
    private double calculateDerivative(double error, double dt) {
        double derivative = (error - previousError) / dt;

        if (Double.isNaN(derivative)) {
            derivative = 0;
        }

        return derivative;
    }
}

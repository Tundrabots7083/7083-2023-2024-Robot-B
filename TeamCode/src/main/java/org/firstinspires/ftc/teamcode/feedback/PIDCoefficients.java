package org.firstinspires.ftc.teamcode.feedback;

/**
 * The PID coefficients used by a PID controller. These are the Kp, Ki and Kd values.
 */
public class PIDCoefficients {

    protected double Kp;
    protected double Ki;
    protected double Kd;

    /**
     * Standard PID coefficients
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change.
     */
    public PIDCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

}

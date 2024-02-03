package org.firstinspires.ftc.teamcode.feedback;

public class PIDCoefficientsEx extends PIDCoefficients {
    public final double Kg;

    /**
     * Standard PID coefficients
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change.
     * @param Kg gravity component, used to ensure the lift can maintain it's position when not
     *           applying additional power.
     */
    public PIDCoefficientsEx(double Kp, double Ki, double Kd, double Kg) {
        super(Kp, Ki, Kd);
        this.Kg = Kg;
    }
}

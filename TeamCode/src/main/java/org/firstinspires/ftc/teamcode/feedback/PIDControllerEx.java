package org.firstinspires.ftc.teamcode.feedback;

/**
 * A PID controller that includes a Kf feed forward component This is useful when there
 * is a heavy component being controlled by the PID controller.
 */
public class PIDControllerEx extends PIDController {
    private FeedForward ff; // Feed forward term, added to the output of the PID calculation

    /**
     * Creates a new PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param Kf constant feed forward term, added to the output of the PID calculation
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, double Kf) {
        super(Kp, Ki, Kd);

        ff = p->Kf;
    }

    /**
     * Creates a new PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param ff feed forward term, added to the output of the PID calculation
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, FeedForward ff) {
        super(Kp, Ki, Kd);

        this.ff = ff;
    }

    /**
     * Calculates the PID controller output.
     *
     * @param reference the target position
     * @param state     current system state
     * @return PID output
     */
    @Override
    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        return power + ff.calculate(reference);
    }

    /**
     * Sets the PID coefficients for the PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change.
     * @param ff feed forward term, added to the output of the PID calculation
     */
    public void setPID(double Kp, double Ki, double Kd, FeedForward ff) {
        super.setPID(Kp, Ki, Kd);
        this.ff = ff;
    }

    /**
     * Sets the PID coefficients for the PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change.
     * @param Kf constant feed forward term, added to the output of the PID calculation
     */
    public void setPID(double Kp, double Ki, double Kd, double Kf) {
        super.setPID(Kp, Ki, Kd);

        ff = p->Kf;
    }
}
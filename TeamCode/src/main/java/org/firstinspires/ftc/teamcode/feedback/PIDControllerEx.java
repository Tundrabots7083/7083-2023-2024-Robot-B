package org.firstinspires.ftc.teamcode.feedback;

/**
 * A PID controller that includes a Kg value to compensate for gravity. This is useful when there
 * is a heavy component being controlled by the PID controller.
 */
public class PIDControllerEx extends PIDController {

    /**
     * Creates a new PID controller.
     *
     * @param coefficients the extended PID coefficients for the PID controller.
     */
    public PIDControllerEx(PIDCoefficientsEx coefficients) {
        super(coefficients);
    }

    /**
     * Calculates the PID controller output.
     *
     * @param reference the target position
     * @param state     current system state
     * @return
     */
    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        PIDCoefficientsEx pidCoefficientsEx = (PIDCoefficientsEx) coefficients;
        return power + pidCoefficientsEx.Kg;
    }
}

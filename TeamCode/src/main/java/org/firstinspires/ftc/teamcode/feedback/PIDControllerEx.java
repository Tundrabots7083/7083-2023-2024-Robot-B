package org.firstinspires.ftc.teamcode.feedback;

public class PIDControllerEx extends PIDController {
    public PIDControllerEx(PIDCoefficientsEx coefficients) {
        super(coefficients);
    }

    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        PIDCoefficientsEx pidCoefficientsEx = (PIDCoefficientsEx) coefficients;
        return power + pidCoefficientsEx.Kg;
    }
}

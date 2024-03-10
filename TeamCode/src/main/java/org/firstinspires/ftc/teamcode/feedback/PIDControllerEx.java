package org.firstinspires.ftc.teamcode.feedback;

public class PIDControllerEx extends PIDController {
    private final double Kg;
    public PIDControllerEx(double Kp, double Ki, double Kd, double Kg) {
        super(Kp, Ki, Kd);

        this.Kg = Kg;
    }

    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        return power + Kg;
    }
}

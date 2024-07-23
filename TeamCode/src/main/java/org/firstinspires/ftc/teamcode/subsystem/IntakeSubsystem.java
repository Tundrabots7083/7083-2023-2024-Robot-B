package org.firstinspires.ftc.teamcode.subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem for the robot. This is responsible for picking up pixels from the field so they
 * may be scored by the <code>ScoringSubsystem</code>.
 */
public class IntakeSubsystem extends SubsystemBaseEx {
    private final Telemetry telemetry;
    private final Lift lift;
    private final Arm arm;
    private final PixelCollector leftPixelCollector;
    private final PixelCollector rightPixelCollector;

    /**
     * Instantiates a new intake subsystem.
     *
     * @param lift                the lift on the robot that can lift or lower the pixel collectors
     * @param arm                 the arm on the robot that can rotate the pixel collectors
     * @param leftPixelCollector  the left pixel collector
     * @param rightPixelCollector the right pixel collector
     * @param telemetry           the telemetry used to output information to the driver station
     */
    public IntakeSubsystem(Lift lift, Arm arm, PixelCollector leftPixelCollector, PixelCollector rightPixelCollector, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.lift = lift;
        this.arm = arm;
        this.leftPixelCollector = leftPixelCollector;
        this.rightPixelCollector = rightPixelCollector;
    }

    /**
     * Have one of the pixel collectors pickup a pixel. This will lower the lift and arm, if not
     * already in the intake position, open the flap door on the pixel collector, and turn on the
     * spinner.
     *
     * @param location the <em>left</em> or <em>right</em> pixel collector
     */
    public void pickupPixel(PixelCollector.Location location) {
        lift.setTarget(Lift.Position.INTAKE);
        arm.setTarget(Arm.Position.INTAKE);

        if (location == PixelCollector.Location.LEFT) {
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        } else {
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.COLLECTING);
        }
    }

    /**
     * Have one of the pixel collectors eject a pixel, typically because the pixel is stuck in
     * the intake mechanism. This will lower the lift and arm, if not already in the intake
     * position, open the flap door on the pixel collector, and turn on the spinner.
     *
     * @param location the <em>left</em> or <em>right</em> pixel collector
     */
    public void ejectPixel(PixelCollector.Location location) {
        lift.setTarget(Lift.Position.INTAKE);
        arm.setTarget(Arm.Position.INTAKE);

        if (location == PixelCollector.Location.LEFT) {
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        } else {
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.DEPOSITING);
        }
    }

    /**
     * Turn off the pixel collector, stopping the spinner and closing the flap door.
     *
     * @param location the <em>left</em> or <em>right</em> pixel collector
     */
    public void stop(PixelCollector.Location location) {
        if (location == PixelCollector.Location.LEFT) {
            leftPixelCollector.setState(PixelCollector.PixelCollectorState.STOPPED);
        } else {
            rightPixelCollector.setState(PixelCollector.PixelCollectorState.STOPPED);
        }
    }

    /**
     * Returns indication as to whether both the arm and lift are at the target positions.
     *
     * @return <code>true</code> if the arm and lift are at the target position;
     * <code>false</code> if one or both is not.
     */
    public boolean isAtTarget() {
        return arm.isAtTarget() && lift.isAtTarget();
    }

    /**
     * Updates the position of the arm, lift and pixel collectors to match the target state.
     */
    @Override
    public void execute() {
        arm.execute();
        lift.execute();
        leftPixelCollector.execute();
        rightPixelCollector.execute();
    }
}

package org.firstinspires.ftc.teamcode.feedback;

/**
 * The MotionProfile class represents a motion profile with specified maximum acceleration, maximum velocity,
 * start position, and end position. It calculates the current position based on the elapsed time since the
 * profile started.
 */
public class MotionProfile {
    private double maxAcceleration;
    private double maxVelocity;
    private double startPosition;
    private double endPosition;
    private long startTime;

    /**
     * Creates a new MotionProfile with the specified maximum acceleration, maximum velocity, start position, and end position.
     * The distance unit used does not matter as long as it is consistent. It can be encoder ticks, inches, feet, etc.
     * The time unit component of the maximum acceleration and maximum velocity constants must be seconds.
     *
     * @param maxAcceleration The maximum acceleration of the motion profile in units per second squared.
     * @param maxVelocity The maximum velocity of the motion profile in units per second.
     * @param startPosition The start position of the motion profile in units.
     * @param endPosition The end position of the motion profile in units.
     */
    public MotionProfile(double maxAcceleration, double maxVelocity, double startPosition, double endPosition) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }

    /**
     * Calculates the current position based on the elapsed time since the motion profile started.
     * 
     * @return The current position.
     */
    public double calculatePosition() {

        if (this.startTime == 0) {
            this.startTime = System.currentTimeMillis();
        }

        double elapsedTime = (System.currentTimeMillis() - this.startTime) / 1000.0; // convert to seconds
        double distance = Math.abs(endPosition - startPosition);
        double direction = Math.signum(endPosition - startPosition);
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
        maxVelocity = maxAcceleration * accelerationDt;
        double decelerationDt = accelerationDt;
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

        double position;
        if (elapsedTime > entireDt) {
            // We've reached the end of the motion profile
            position = distance;
        } else if (elapsedTime < accelerationDt) {
            // Acceleration Phase
            position = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        } else if (elapsedTime < decelerationTime) {
            // Cruise Phase
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            double cruiseCurrentDt = elapsedTime - accelerationDt;
            position = accelerationDistance + maxVelocity * cruiseCurrentDt;
        } else {
            // Deceleration Phase
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            cruiseDistance = maxVelocity * cruiseDt;
            double decelerationCurrentTime = elapsedTime - decelerationTime;
            position = accelerationDistance + cruiseDistance + maxVelocity * decelerationCurrentTime - 0.5 * maxAcceleration * Math.pow(decelerationCurrentTime, 2);
        }

        return startPosition + direction * position;
    }
}
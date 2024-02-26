package org.firstinspires.ftc.teamcode.feedback;

/**
 * The MotionProfile class represents a motion profile with specified maximum acceleration, maximum velocity,
 * start position, and end position. It calculates the current position based on the elapsed time since the
 * profile started.
 */
public class MotionProfile {
    private final double maxAcceleration;
    private final double maxVelocity;
    private final double startPosition;
    private double endPosition;
    private long startTime;

    /**
     * Creates a new MotionProfile with the specified maximum acceleration, maximum velocity, start position, and end position.
     * The distance unit used does not matter as long as it is consistent. It can be encoder ticks, inches, feet, etc.
     * The time unit component of the maximum acceleration and maximum velocity constants must be seconds.
     *
     * @param maxAcceleration The maximum acceleration of the motion profile in units per second squared.
     * @param maxVelocity     The maximum velocity of the motion profile in units per second.
     * @param startPosition   The start position of the motion profile in units.
     * @param endPosition     The end position of the motion profile in units.
     */
    public MotionProfile(double maxAcceleration, double maxVelocity, double startPosition, double endPosition) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.startPosition = startPosition;
        this.endPosition = endPosition;

        if (this.startPosition == this.endPosition) {
            this.endPosition += 1;
        }
    }

    /**
     * The amount of time spent accelerating.
     *
     * @return the amount of time spent accelerating.
     */
    private double getAccelerationDt() {
        return maxVelocity / maxAcceleration;
    }

    /**
     * The amount of time spent decelerating.
     *
     * @return the amount of time spent decelerating.
     */
    private double getDecelerationDt() {
        return getAccelerationDt();
    }

    /**
     * The amount of time spent cruising.
     *
     * @return the amount of time spent cruising.
     */
    private double getCruiseDt() {
        double distance = Math.abs(endPosition - startPosition);
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(getAccelerationDt(), 2);
        double cruiseDistance = distance - 2 * accelerationDistance;
        double localMaxVelocity = maxAcceleration * getAccelerationDt();
        return cruiseDistance / localMaxVelocity;
    }

    private double getEntireDt() {
        return getAccelerationDt() + getDecelerationDt() + getCruiseDt();
    }

    private double getElapsedTime() {
        if (this.startTime == 0) {
            this.startTime = System.currentTimeMillis();
        }
        return (System.currentTimeMillis() - this.startTime) / 1000.0; // convert to seconds
    }

    /**
     * The robot has reached the target position.
     *
     * @return <code>true</code> if the robot has reached the target position;
     * <code>false</code> if it has not yet reached the target position.
     */
    public boolean isAtEnd() {
        return getElapsedTime() > getEntireDt();
    }

    /**
     * The robot is accelerating.
     *
     * @return <code>true</code> if the robot is accelerating;
     * <code>false</code> if the robot is not accelerating.
     */
    public boolean isAccelerating() {
        return getElapsedTime() < getAccelerationDt();
    }

    /**
     * The robot is cruising (at a steady speed).
     *
     * @return <code>true</code> if the robot is cruising;
     * <code>false</code> if the robot is not cruising.
     */
    public boolean isCruising() {
        double decelerationTime = getAccelerationDt() + getCruiseDt();
        return getElapsedTime() < decelerationTime;
    }

    /**
     * The robot is decelerating.
     *
     * @return the robot is decelerating.
     */
    public boolean isDecelerating() {
        return !isAtEnd() && !isAccelerating() && !isCruising();
    }

    /**
     * Calculates the current position based on the elapsed time since the motion profile started.
     *
     * @return The current position.
     */
    public double calculatePosition() {
        double elapsedTime = getElapsedTime();

        double localMaxVelocity = maxAcceleration * getAccelerationDt();

        double position;
        if (isAtEnd()) {
            // We've reached the end of the motion profile
            position = Math.abs(endPosition - startPosition);
        } else if (isAccelerating()) {
            // Acceleration Phase
            position = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        } else if (isCruising()) {
            // Cruise Phase
            double accelerationDistance = 0.5 * maxAcceleration * Math.pow(getAccelerationDt(), 2);
            double cruiseCurrentDt = elapsedTime - getAccelerationDt();
            position = accelerationDistance + localMaxVelocity * cruiseCurrentDt;
        } else {
            // Deceleration Phase
            double accelerationDistance = 0.5 * maxAcceleration * Math.pow(getAccelerationDt(), 2);
            double cruiseDistance = localMaxVelocity * getCruiseDt();
            double decelerationTime = getAccelerationDt() + getCruiseDt();
            double decelerationCurrentTime = elapsedTime - decelerationTime;
            position = accelerationDistance + cruiseDistance + localMaxVelocity * decelerationCurrentTime - 0.5 * maxAcceleration * Math.pow(decelerationCurrentTime, 2);
        }

        double direction = Math.signum(endPosition - startPosition);
        return startPosition + direction * position;
    }
}
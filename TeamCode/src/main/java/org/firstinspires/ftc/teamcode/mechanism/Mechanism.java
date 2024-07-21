package org.firstinspires.ftc.teamcode.mechanism;

/**
 * Mechanism is the base class for all mechanisms on the robot.
 */
public interface Mechanism {
    default void execute() {
        // NO-OP
    }
}
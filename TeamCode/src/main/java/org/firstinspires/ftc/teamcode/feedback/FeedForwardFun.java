package org.firstinspires.ftc.teamcode.feedback;

/**
 * Interface for calculating the feed forward value for a PIDF controller.
 */
@FunctionalInterface
public interface FeedForwardFun {
    /**
     * Calculate the feed forward value.
     *
     * @param Kf custom, position-dependent feedforward (e.g., a gravity term for arms)
     * @return the feed forward value for the PIDF controller.
     */
    double ff(double Kf);
}

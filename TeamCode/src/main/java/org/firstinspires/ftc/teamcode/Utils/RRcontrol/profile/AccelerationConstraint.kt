package org.firstinspires.ftc.teamcode.Utils.RRcontrol.profile

/**
 * Motion profile acceleration constraint.
 */
fun interface AccelerationConstraint {

    /**
     * Returns the maximum profile acceleration at displacement [s].
     */
    operator fun get(s: Double): Double
}

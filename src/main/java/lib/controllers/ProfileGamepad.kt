package lib.controllers

import lib.profiles.DriverProfile
import kotlin.math.pow
import kotlin.math.absoluteValue

/**
 * A Gamepad that supports having a [DriverProfile] that affects its input mapping.
 *
 * Raw versions of joystick axes (un-scaled and un-multiplied) simply have
 * `Raw` as a suffix to their regular method.
 *
 * @author Matthew Clark
 *
 * @see Gamepad
 * @see DriverProfile
 */
interface ProfileGamepad : Gamepad {
    
    // Could get removed honostly
    /**
     * The specific profile that is affecting this controller's input mapping
     *
     * @see DriverProfile.currentProfile
     */
    val profile: DriverProfile.Profile

    /**
     * The raw value of the left joystick axis in the X direction (un-scaled and un-multiplied)
     *
     * @see leftXAxis
     */
    val leftXAxisRaw: Double

    /**
     * The raw value of the left joystick axis in the Y direction (un-scaled and un-multiplied)
     *
     * @see leftYAxis
     */
    val leftYAxisRaw: Double

    /**
     * The raw value of the right joystick axis in the X direction (un-scaled and un-multiplied)
     *
     * @see rightXAxis
     */
    val rightXAxisRaw: Double

    /**
     * The raw value of the right joystick axis in the Y direction (un-scaled and un-multiplied)
     *
     * @see rightYAxis
     */
    val rightYAxisRaw: Double
}

/**
 * Raises a base to an exponent *specifically to scale its value.*
 * 
 * It will retain its sign, and will give proper (but negative) values
 * for all negative bases for all fractional exponents
 */
fun Double.powScale(exp: Double): Double {
    // (-1.0) ** 2.0 > 0                        <- bad
    // (-1.0) * |-1.0| ** (2.0 - 1) < 0         <- good
    // (-1.0) ** (3.0 / 2.0) => NaN             <- bad
    // (-1.0) * |-1.0| ** (2.0 / 3.0 - 1) == 1  <- good
    return this * this.absoluteValue.pow(exp - 1.0)
}

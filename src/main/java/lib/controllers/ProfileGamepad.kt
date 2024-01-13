package lib.controllers

import lib.profiles.DriverProfile
import kotlin.math.pow
import kotlin.math.absoluteValue

/**
 * A Gamepad that supports having a [DriverProfile.Profile] that affects its input mapping.
 *
 * Raw versions of joystick axes (un-scaled and un-multiplied) simply have
 * `Raw` as a suffix to their regular method.
 *
 * @author Matthew Clark
 *
 * @see Gamepad
 * @see DriverProfile
 */
@Deprecated("See ProfileController")
interface ProfileGamepad : Gamepad {
    /**
     * The active profile for the gamepad
     *
     * @see DriverProfile
     */
    val profile: DriverProfile.Profile
        get() = DriverProfile.currentProfile

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

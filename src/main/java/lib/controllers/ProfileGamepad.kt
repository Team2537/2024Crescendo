package lib.controllers

import lib.profiles.DriverProfile

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
    /**
     * The specific profile that is affecting this controller's input mapping
     *
     * @see DriverProfile
     */
    var profile: DriverProfile

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

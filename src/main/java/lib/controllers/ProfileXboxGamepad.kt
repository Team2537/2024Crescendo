package lib.controllers

import lib.powScale
import lib.profiles.DriverProfile

/**
 * An xbox gamepad that supports being given a specific [DriverProfile]
 *
 * The profile of a [ProfileXboxGamepad] can be changed or retrieved at any time
 * through its [ProfileXboxGamepad.profile] property. Any properties that are
 * changed by the profile can be obtained as raw by a simple 'Raw' suffix
 * (e.g. [ProfileXboxGamepad.leftXAxisRaw])
 *
 * @author Matthew Clark
 *
 * @see DriverProfile
 * @see XboxGamepad
 */

@Deprecated(
    "Use ProfileController in combination with XboxGamepad",
    level = DeprecationLevel.HIDDEN,
)
class ProfileXboxGamepad(port: Int) : XboxGamepad(port), ProfileGamepad {
    private val leftXInverse: Double
        get() = if (profile.invertLeftX) -1.0 else 1.0

    private val leftYInverse: Double
        get() = if (profile.invertLeftY) -1.0 else 1.0

    private val rightXInverse: Double
        get() = if (profile.invertRightX) -1.0 else 1.0

    private val rightYInverse: Double
        get() = if (profile.invertRightY) -1.0 else 1.0

    override val leftXAxis: Double
        get() = leftXInverse * super.leftXAxis.powScale(profile.leftPowerScale)

    override val leftXAxisRaw: Double
        get() = super.leftXAxis

    override val leftYAxis: Double
        get() = leftYInverse * super.leftYAxis.powScale(profile.leftPowerScale)

    override val leftYAxisRaw: Double
        get() = super.leftYAxis

    override val rightXAxis: Double
        get() = rightXInverse * super.rightXAxis.powScale(profile.rightPowerScale)

    override val rightXAxisRaw: Double
        get() = super.rightXAxis

    override val rightYAxis: Double
        get() = rightYInverse * super.rightYAxis.powScale(profile.rightPowerScale)

    override val rightYAxisRaw: Double
        get() = super.rightYAxis
}

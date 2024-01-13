package lib.controllers

import lib.profiles.DriverProfile
import lib.profiles.defaultDriverProfile
import kotlin.math.pow

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
class ProfileXboxGamepad(port: Int) : XboxGamepad(port), ProfileGamepad {
    private val leftXInverse: Double
        get() = if (profile.invertLeftX) -1.0 else 1.0

    private val leftYInverse: Double
        get() = if (profile.invertLeftY) -1.0 else 1.0

    private val rightXInverse: Double
        get() = if (profile.invertRightX) -1.0 else 1.0

    private val rightYInverse: Double
        get() = if (profile.invertRightY) -1.0 else 1.0

    private var _profile: DriverProfile

    override var profile: DriverProfile
        get() = _profile
        set(value) {
            _profile = value
        }

    init {
        _profile = defaultDriverProfile()
    }

    override val leftXAxis: Double
        get() = leftXInverse * leftXAxis.pow(profile.leftPowerScale)

    override val leftXAxisRaw: Double
        get() = super.leftXAxis

    override val leftYAxis: Double
        get() = leftYInverse * leftYAxis.pow(profile.leftPowerScale)

    override val leftYAxisRaw: Double
        get() = super.leftYAxis

    override val rightXAxis: Double
        get() = rightXInverse * super.rightXAxis.pow(profile.rightPowerScale)

    override val rightXAxisRaw: Double
        get() = super.rightXAxis

    override val rightYAxis: Double
        get() = rightYInverse * super.rightYAxis.pow(profile.rightPowerScale)

    override val rightYAxisRaw: Double
        get() = super.rightYAxis
}

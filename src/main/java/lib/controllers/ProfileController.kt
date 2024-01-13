package lib.controllers

import edu.wpi.first.wpilibj2.command.button.Trigger
import lib.powScale
import lib.profiles.DriverProfile
import lib.toTrigger

/**
 * Wrapper class for any [Gamepad] that applies the currently active [Profile][DriverProfile.Profile]
 *
 * @see Gamepad
 * @see DriverProfile
 */
class ProfileController(private val gamepad: Gamepad) {
    private val profile: DriverProfile.Profile
        get() = DriverProfile.currentProfile

    private val leftXInverse: Double
        get() = if (profile.invertLeftX) -1.0 else 1.0

    private val leftYInverse: Double
        get() = if (profile.invertLeftY) -1.0 else 1.0

    private val rightXInverse: Double
        get() = if (profile.invertRightX) -1.0 else 1.0

    private val rightYInverse: Double
        get() = if (profile.invertRightY) -1.0 else 1.0

    /**
     * The raw value of the left joystick axis in the X direction (un-scaled and un-multiplied)
     *
     * @see leftXAxis
     */
    val leftXAxisRaw: Double
        get() = gamepad.leftXAxis

    /**
     * The raw value of the left joystick axis in the Y direction (un-scaled and un-multiplied)
     *
     * @see leftYAxis
     */
    val leftYAxisRaw: Double
        get() = gamepad.leftYAxis

    /**
     * The raw value of the right joystick axis in the X direction (un-scaled and un-multiplied)
     *
     * @see rightXAxis
     */
    val rightXAxisRaw: Double
        get() = gamepad.rightXAxis

    /**
     * The raw value of the right joystick axis in the Y direction (un-scaled and un-multiplied)
     *
     * @see rightYAxis
     */
    val rightYAxisRaw: Double
        get() = gamepad.rightYAxis

    val leftXAxis: Double
        get() = leftXInverse * gamepad.leftXAxis.powScale(profile.leftPowerScale)

    val leftYAxis: Double
        get() = leftYInverse * gamepad.leftYAxis.powScale(profile.leftPowerScale)
    
    val rightXAxis: Double
        get() = rightXInverse * gamepad.rightXAxis.powScale(profile.rightPowerScale)

    val rightYAxis: Double
        get() = rightYInverse * gamepad.rightYAxis.powScale(profile.rightPowerScale)


    val leftTriggerAxis: Double
        get() = gamepad.leftTriggerAxis
    val rightTriggerAxis: Double
        get() = gamepad.rightTriggerAxis

    /**
     * Converting all boolean properties to triggers
     * @see toTrigger
     */
    val aButton: Trigger = gamepad.aButton.toTrigger()
    val bButton: Trigger = gamepad.bButton.toTrigger()
    val xButton: Trigger = gamepad.xButton.toTrigger()
    val yButton: Trigger = gamepad.yButton.toTrigger()
    val leftJoystickButton: Trigger = gamepad.leftJoystickButton.toTrigger()
    val rightJoystickButton: Trigger = gamepad.rightJoystickButton.toTrigger()
    val leftShoulderButton: Trigger = gamepad.leftShoulderButton.toTrigger()
    val rightShoulderButton: Trigger = gamepad.rightShoulderButton.toTrigger()
    val dPadUp: Trigger = gamepad.dPadUp.toTrigger()
    val dPadDown: Trigger = gamepad.dPadDown.toTrigger()
    val dPadLeft: Trigger = gamepad.dPadLeft.toTrigger()
    val dPadRight: Trigger = gamepad.dPadRight.toTrigger()
    val startButton: Trigger = gamepad.startButton.toTrigger()
    val selectButton: Trigger = gamepad.selectButton.toTrigger()
}
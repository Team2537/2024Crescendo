package lib.profiles

import kotlinx.serialization.Serializable
import lib.profiles.DriverProfile.Presets
import java.util.function.Consumer

/**
 * A set of preferences for drivers.
 *
 * Each profile has various fields that change behaviour, such as inverting
 * certain axes. The actual effect that a profile has on a
 * [Gamepad][lib.controllers.Gamepad] is implemented by
 * [ProfileController][lib.controllers.ProfileController].
 *
 * A list of preset profiles can be found in the [Presets] enum.
 *
 * @author Matthew Clark
 * @author Micah Rao
 *
 * @see Presets
 * @see lib.controllers.Gamepad
 * @see lib.controllers.ProfileController
 */
@Serializable
data class DriverProfile (
    val friendlyName: String,
    val invertLeftX: Boolean,
    val invertLeftY: Boolean,
    val invertRightX: Boolean,
    val invertRightY: Boolean,
    val southpaw: Boolean,
    val leftPowerScale: Double,
    val rightPowerScale: Double,
) {

    /**
     * The list of preset profiles that have already been created.
     *
     * @author Micah Rao
     * @author Matthew Clark
     */
    enum class Presets(val profile: DriverProfile) {
        DEFAULT (
            DriverProfile(
                friendlyName = "Default",
                invertLeftX = false,
                invertLeftY = false,
                invertRightX = false,
                invertRightY = false,
                southpaw = false,
                leftPowerScale = 1.0,
                rightPowerScale = 1.0,
            )
        ), // TODO: add more driver profiles
        ;

        /** @suppress */
        companion object {
            fun forEach(function: (DriverProfile) -> Unit) {
                entries.forEach {
                    function(it.profile)
                }
            }

            fun forEach(action: Consumer<DriverProfile>) {
                entries.forEach {
                    action.accept(it.profile)
                }
            }
        }
    }

    /** @suppress */
    companion object {
        // Only ever change this variable to set the default profile
        val default: DriverProfile = Presets.DEFAULT.profile
    }
}

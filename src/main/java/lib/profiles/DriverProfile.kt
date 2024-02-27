package lib.profiles

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

/**
 * Driver Profile manager, should be eager initialized when the robot starts
 * @see frc.robot.RobotContainer
 */
object DriverProfile {
    private val driverProfileChooser =
        SendableChooser<Profile>().apply {
            Profile.entries.forEach { addOption(it.friendlyName, it) }
            setDefaultOption(Profile.default.friendlyName, Profile.default)
        }

    /**
     * Gets the current selected profile from the [profile chooser][driverProfileChooser].
     */
    val currentProfile: Profile
        // Elvis operator for null-safety
        get() = driverProfileChooser.selected ?: Profile.default

    init {
        val tab = Shuffleboard.getTab("Driver Profiles")
        tab.add(driverProfileChooser)
    }

    enum class Profile(
        val friendlyName: String,
        val invertLeftX: Boolean,
        val invertLeftY: Boolean,
        val invertRightX: Boolean,
        val invertRightY: Boolean,
        val southpaw: Boolean,
        val leftPowerScale: Double,
        val rightPowerScale: Double,
    ) {
        DEFAULT_PROFILE(
            friendlyName = "Default",
            invertLeftX = false,
            invertLeftY = false,
            invertRightX = false,
            invertRightY = false,
            southpaw = false,
            leftPowerScale = 1.0,
            rightPowerScale = 1.0,
        ), // TODO: add more driver profiles
        ;

        companion object {
            // Only ever change this variable to set the default profile
            val default: Profile = DEFAULT_PROFILE
        }
    }
}

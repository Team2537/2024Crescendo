package lib.profiles

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser

/**
 * Driver Profile manager, should be eager initialized when the robot starts
 * @see frc.robot.RobotContainer
 */
object DriverProfile {
    private val driverProfileChooser = SendableChooser<Profile>().apply {
        Profile.values().forEach { addOption(it.name, it) }
        setDefaultOption(Profile.default.name, Profile.default)
    }

    /**
     * Gets the current selected profile from the [profile chooser][driverProfileChooser].
     * Elvis operator creates null-safety.
     */
    val currentProfile: Profile
        get() = driverProfileChooser.selected ?: Profile.default

    init {
        val tab = Shuffleboard.getTab("Driver Profiles")
        tab.add(driverProfileChooser)
    }

    enum class Profile(
        val invertLeftX: Boolean,
        val invertLeftY: Boolean,
        val invertRightX: Boolean,
        val invertRightY: Boolean,
        val southpaw: Boolean,
        val leftPowerScale: Double,
        val rightPowerScale: Double,
    ) {
        DEFAULT_PROFILE(
            false,
            false,
            false,
            false,
            false,
            1.0,
            1.0,
        ), // TODO: add more driver profiles
        ;

        companion object {
            val default: Profile = DEFAULT_PROFILE
        }
    }
}
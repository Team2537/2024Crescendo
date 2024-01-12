package frc.robot.util

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.RobotContainer

/**
 * The default driver profile.
 *
 * This is the default controls for a controller, and [RobotContainer]
 * will use these settings initially via [RobotContainer.configureBindings].
 *
 * The current controls this profile specifies were directly taken from the
 * implementation of RobotContainer.configureBindings at the time of writing.
 *
 * @Deprecated [DriverProfile] has been replaced by [lib.controllers.DriverProfile]
 */
@Deprecated("New DriverProfile in lib")
object DefaultDriverProfile : DriverProfile {
    override fun applyPreference(controller: CommandXboxController) {
        controller.b().onTrue(RobotContainer.drivebase.runOnce { RobotContainer.drivebase.zeroGyro() })
        controller.a().toggleOnTrue(RobotContainer.trackTarget)
        controller.x().toggleOnTrue(RobotContainer.cornerSpin)
    }
}

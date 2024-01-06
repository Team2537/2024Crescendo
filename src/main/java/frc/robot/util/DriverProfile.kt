package frc.robot.util

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

/**
 * A set of controls and preferences on how a controller is used.
 * This includes button-bindings, sensitivities, etc.
 *
 * @author Matthew Clark
 * @see DefaultDriverProfile
 */
interface DriverProfile {
    fun applyPreference(controller : CommandXboxController) // TODO: change with abstract controller
}
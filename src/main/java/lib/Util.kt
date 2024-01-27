package lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.SwerveSubsystem
import kotlinx.serialization.json.Json
import kotlinx.serialization.serializer
import lib.zones.Zone
import lib.zones.Zones
import kotlin.math.absoluteValue
import kotlin.math.atan
import kotlin.math.pow

/**
 * Temporary storage for extension functions and other useful tools
 * TODO: Potentially sort these functions into other files later on
 */

/**
 * Raises a base to an exponent *specifically to scale its value.*
 *
 * It will retain its sign, and will give proper (but negative) values
 * for all negative bases for all fractional exponents
 */
fun Double.powScale(exp: Double): Double {
    // (-1.0) ** 2.0 > 0                        <- bad
    // (-1.0) * |-1.0| ** (2.0 - 1) < 0         <- good
    // (-1.0) ** (3.0 / 2.0) => NaN             <- bad
    // (-1.0) * |-1.0| ** (2.0 / 3.0 - 1) == 1  <- good
    return this * this.absoluteValue.pow(exp - 1.0)
}

/**
 * Converts a boolean to a [Trigger] object
 */
fun Boolean.toTrigger(): Trigger {
    return Trigger { this }
}

inline fun <reified T> Json.encodeToString(data: T): String {
    return encodeToString(serializersModule.serializer(), data)
}

/**
 * Returns a trigger that is `true` while the position is inside the zone, and
 * `false` otherwise.
 *
 * @return A trigger that checks against a specific zone
 */
inline fun zoneTrigger(tag: String, crossinline position: () -> Pose2d = { SwerveSubsystem.getPose() }): Trigger {
    return zoneTrigger(Zones[tag], position)
}

/**
 * Returns a trigger that is `true` while the position is inside the zone, and
 * `false` otherwise.
 *
 * @return A trigger that checks against a specific zone
 */
inline fun zoneTrigger(zone: Zone, crossinline position: () -> Pose2d = { SwerveSubsystem.getPose() }): Trigger {
    return Trigger { Zones[position.invoke()] == zone }
}

private fun getAngleToShootRaw(
    targetHeight: Double,
    targetDistance: Double,
    launcherHeight: Double,
    epsilon: Double = 0.001
): Double {
    return atan((targetHeight - launcherHeight) / targetDistance) + epsilon * 9.8 * targetDistance
}

/**
 * Gets the pitch angle required (approximately) to hit a target at a given distance.
 *
 * This function assumes that the projectile will travel in a straight line, while
 * giving a minor positive offset based on the distance to account for gravity.
 *
 * @param targetHeight The height of the target
 * @param targetDistance The distance from the launcher to the target
 * @param launcherHeight The height of the launcher
 * @param epsilon How much to scale the distance/gravity offset
 *
 * @return An angle of elevation to aim at relative to the horizontal
 */
fun getPitchTowardsTarget(
    targetHeight: Measure<Distance>,
    targetDistance: Measure<Distance>,
    launcherHeight: Measure<Distance>,
    epsilon: Double = 0.001
): Measure<Angle> {
    return Units.Radians.of(
        getAngleToShootRaw(
            targetHeight.`in`(Units.Meters),
            targetDistance.`in`(Units.Meters),
            launcherHeight.`in`(Units.Meters),
            epsilon)
    )
}

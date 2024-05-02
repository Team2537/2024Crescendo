package lib

import com.pathplanner.lib.util.GeometryUtil
import com.revrobotics.CANSparkBase
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.SwerveSubsystem
import kotlinx.serialization.json.Json
import kotlinx.serialization.serializer
import lib.math.units.Span
import lib.math.units.into
import lib.zones.Zone
import lib.zones.Zones
import swervelib.SwerveDrive
import kotlin.math.abs
import kotlin.math.absoluteValue
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
fun Boolean.toTrigger(): Trigger = Trigger { this }

/**
 *  Sets the velocity reference of a REV motor controller.
 *
 *  @param vel the velocity to set the motor to.
 */
fun CANSparkBase.setVelocity(vel: Double) {
    this.pidController.setReference(vel, CANSparkBase.ControlType.kVelocity)
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
inline fun zoneTrigger(tag: String, crossinline position: () -> Pose2d = { SwerveSubsystem.pose }): Trigger {
    return zoneTrigger(Zones[tag], position)
}

/**
 * Returns a trigger that is `true` while the position is inside the zone, and
 * `false` otherwise.
 *
 * @return A trigger that checks against a specific zone
 */
inline fun zoneTrigger(zone: Zone, crossinline position: () -> Pose2d = { SwerveSubsystem.pose }): Trigger {
    return Trigger { Zones[position.invoke()] == zone }
}

fun Double.near(target: Double, error: Double): Boolean {
    return abs(this - target) < error
}

fun InterpolatingDoubleTreeMap.putMap(map: HashMap<Double, Double>) {
    for (entry in map.entries) {
        val key = entry.key
        val value = entry.value
        this.put(key, value)
    }
}

/**
 * Sets the position reference of a REV motor controller.
 *
 * @param pos the position to set the motor to.
 */
fun CANSparkBase.setPosition(pos: Double) {
    this.pidController.setReference(pos, CANSparkBase.ControlType.kPosition)
}

fun Translation2d.flip(): Translation2d {
    return GeometryUtil.flipFieldPosition(this)
}

fun SwerveDrive.evilGetHeading(): Double {
    return javaClass.getDeclaredField("lastHeadingRadians").let {
        it.isAccessible = true
        val value = it.getDouble(this)
        return@let value
    }
}

fun calculateAngle(distance: Span): Double {
    val gx = distance into Units.Inches
    return (-0.33 * gx) + 78.5
}


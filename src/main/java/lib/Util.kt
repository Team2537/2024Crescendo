package lib

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlinx.serialization.json.Json
import kotlinx.serialization.serializer
import lib.zones.Zone
import lib.math.units.Span
import lib.math.units.into
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
@Deprecated("It will be rewritten")
inline fun zoneTrigger(tag: String, crossinline position: () -> Pose2d): Trigger {
    return zoneTrigger(Zones[tag], position)
}

/**
 * Returns a trigger that is `true` while the position is inside the zone, and
 * `false` otherwise.
 *
 * @return A trigger that checks against a specific zone
 */
@Deprecated("It will be rewritten")
inline fun zoneTrigger(zone: Zone, crossinline position: () -> Pose2d): Trigger {
    return Trigger { Zones[position.invoke()] == zone }
}

fun Double.near(target: Double, error: Double): Boolean{
    return abs(this - target) < error
}

fun InterpolatingDoubleTreeMap.putMap(map: HashMap<Double, Double>){
    for(entry in map.entries){
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

fun Pose2d.near(other: Pose2d, epsilon: Double = 1E-9): Boolean {
    return this.translation.near(other.translation, epsilon) && this.rotation.near(other.rotation, epsilon)
}

fun Rotation2d.near(other: Rotation2d, epsilon: Double = 1E-9): Boolean {
    return this.cos.near(other.cos, epsilon) && this.sin.near(other.sin, epsilon)
}

fun Translation2d.near(other: Translation2d, epsilon: Double = 1E-9): Boolean {
    return this.x.near(other.x, epsilon) && this.y.near(other.y, epsilon)
}

fun Pose2d.flip() = Pose2d(this.translation.flip(), this.rotation.flip())

fun Translation2d.flip() = Translation2d(16.54 - this.x, this.y)

fun Rotation2d.flip(): Rotation2d = this.rotateBy(Rotation2d.fromDegrees(180.0))

fun Pose3d.flip() = Pose3d(this.translation.flip(), this.rotation.flip())

fun Translation3d.flip() = Translation3d(16.54 - this.x, this.y, this.z)

fun Rotation3d.flip(): Rotation3d = this.rotateBy(Rotation3d(0.0, 0.0, Math.PI))

fun Gamepiece.flip() = Gamepiece(this.first.flip(), this.second)

fun Transform3d.toTransform2d() = Transform2d(this.translation.toTranslation2d(), this.rotation.toRotation2d())

operator fun Trigger.not(): Trigger {
    return this.negate()
}

fun Command.debug(msg: String = "${this.name} is running"): Command {
    return this.deadlineWith(PrintCommand(msg).repeatedly())
}
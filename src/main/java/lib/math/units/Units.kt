package lib.math.units

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.*
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.*

infix fun <U : Unit<U>> Measure<U>.into(unit: U): Double {
    return this.`in`(unit)
}

/**
 * Negates a [Measure] through its [Measure.negate] method
 *
 * @return The resulting measure
 */
operator fun <U : Unit<U>?> Measure<U>.unaryMinus(): Measure<U> {
    return this.negate()
}

/**
 * Does nothing to a measure. If this measure was mutable, it will return a mutable copy
 *
 * @return The resulting measure
 */
operator fun <U : Unit<U>?> Measure<U>.unaryPlus(): Measure<U> {
    return if(this is ImmutableMeasure) this else mutableCopy()
}

var CANSparkBase.velocity: RotationVelocity
    get() = encoder.velocity.rpm
    set(value) {
        pidController.setReference(value into RPM, CANSparkBase.ControlType.kVelocity)
    }

val Translation3d.forward: Span
    get() = x.meters

val Translation3d.backwards: Span
    get() = -forward

val Translation3d.left: Span
    get() = y.meters

val Translation3d.right: Span
    get() = -left

val Translation3d.up: Span
    get() = z.meters

val Translation3d.down: Span
    get() = -up // Why? consistency

val Rotation3d.roll: Rotation
    get() = x.radians

val Rotation3d.pitch: Rotation
    get() = y.radians

val Rotation3d.yaw: Rotation
    get() = z.radians

val Translation2d.forward: Span
    get() = x.meters

val Translation2d.backwards: Span
    get() = -forward

val Translation2d.left: Span
    get() = y.meters

val Translation2d.right: Span
    get() = -left

val Rotation2d.measure: Rotation
    get() = radians.radians

val Pose3d.forward: Span
    get() = translation.forward

val Pose3d.backwards: Span
    get() = translation.backwards

val Pose3d.left: Span
    get() = translation.left

val Pose3d.right: Span
    get() = translation.right

val Pose3d.up: Span
    get() = translation.up

val Pose3d.down: Span
    get() = translation.down

val Pose3d.roll: Rotation
    get() = rotation.roll

val Pose3d.pitch: Rotation
    get() = rotation.pitch

val Pose3d.yaw: Rotation
    get() = rotation.yaw

val Pose2d.forward: Span
    get() = translation.forward

val Pose2d.backwards: Span
    get() = translation.backwards

val Pose2d.left: Span
    get() = translation.left

val Pose2d.right: Span
    get() = translation.right

val Pose2d.spin: Rotation
    get() = rotation.measure

typealias RotationVelocity = Measure<Velocity<Angle>>
typealias SpanVelocity = Measure<Velocity<Distance>>
typealias Speed = SpanVelocity
typealias Span = Measure<Distance>
typealias Rotation = Measure<Angle>
typealias TimeSpan = Measure<Time>

inline val Double.meters: Span
    get() = Meters.of(this)

inline val Double.feet: Span
    get() = Feet.of(this)

inline val Double.centimeters: Span
    get() = Centimeters.of(this)

inline val Double.millimeters: Span
    get() = Millimeters.of(this)

inline val Double.radians: Rotation
    get() = Radians.of(this)

inline val Double.degrees: Rotation
    get() = Degrees.of(this)

inline val Double.rotations: Rotation
    get() = Rotations.of(this)

inline val Double.revolutions: Rotation
    get() = Revolutions.of(this)

inline val Double.metersPerSecond: SpanVelocity
    get() = MetersPerSecond.of(this)

inline val Double.feetPerSecond: SpanVelocity
    get() = FeetPerSecond.of(this)

inline val Double.inchesPerSecond: SpanVelocity
    get() = InchesPerSecond.of(this)
inline val Double.degreesPerSecond: RotationVelocity
    get() = DegreesPerSecond.of(this)

inline val Double.radiansPerSecond: RotationVelocity
    get() = RadiansPerSecond.of(this)

inline val Double.rpm: RotationVelocity
    get() = RPM.of(this)

inline val Double.seconds: TimeSpan
    get() = Seconds.of(this)

inline val Double.minutes: TimeSpan
    get() = Minutes.of(this)

inline val Double.milliseconds: TimeSpan
    get() = Milliseconds.of(this)

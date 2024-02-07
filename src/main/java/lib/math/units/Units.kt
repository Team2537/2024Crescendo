package lib.math.units

import com.revrobotics.CANSparkBase
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.RPM


var CANSparkBase.velocity: RotationVelocity
    get() = encoder.velocity.rpm
    set(value) {
        pidController.setReference(value.`in`(RPM), CANSparkBase.ControlType.kVelocity)
    }




typealias RotationVelocity = Measure<Velocity<Angle>>
typealias SpanVelocity = Measure<Velocity<Distance>>
typealias Speed = SpanVelocity
typealias Span = Measure<Distance>
typealias Rotation = Measure<Angle>

inline val Double.meters: Span
    get() = Units.Meters.of(this)

inline val Double.feet: Span
    get() = Units.Feet.of(this)

inline val Double.centimeters: Span
    get() = Units.Centimeters.of(this)

inline val Double.millimeters: Span
    get() = Units.Millimeters.of(this)

inline val Double.radians: Rotation
    get() = Units.Radians.of(this)

inline val Double.degrees: Rotation
    get() = Units.Degrees.of(this)

inline val Double.rotations: Rotation
    get() = Units.Rotations.of(this)

inline val Double.revolutions: Rotation
    get() = Units.Revolutions.of(this)

inline val Double.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this)

inline val Double.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this)

inline val Double.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this)
inline val Double.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this)

inline val Double.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this)

inline val Double.rpm: RotationVelocity
    get() = Units.RPM.of(this)

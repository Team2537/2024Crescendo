/**
 * Single-file library for improving the usability of WPILib's units API.
 *
 * @author Matthew Clark
 * @author Falon Clark
 *
 * @since 6/10/24
 */

package lib.math.units

import edu.wpi.first.units.*
import edu.wpi.first.units.Unit

inline val Number.inMeters: Measure<Distance>
    get() = Units.Meters.of(this.toDouble())

inline val Number.inFeet: Measure<Distance>
    get() = Units.Feet.of(this.toDouble())

inline val Number.inCentimeters: Measure<Distance>
    get() = Units.Centimeters.of(this.toDouble())

inline val Number.inMillimeters: Measure<Distance>
    get() = Units.Millimeters.of(this.toDouble())

inline val Number.inRadians: Measure<Angle>
    get() = Units.Radians.of(this.toDouble())

inline val Number.inDegrees: Measure<Angle>
    get() = Units.Degrees.of(this.toDouble())

inline val Number.inRotations: Measure<Angle>
    get() = Units.Rotations.of(this.toDouble())

inline val Number.inRevolutions: Measure<Angle>
    get() = Units.Revolutions.of(this.toDouble())

inline val Number.inMPS: Measure<Velocity<Distance>>
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Number.inFPS: Measure<Velocity<Distance>>
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Number.inIPS: Measure<Velocity<Distance>>
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Number.inDPS: Measure<Velocity<Angle>>
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Number.inRPS: Measure<Velocity<Angle>>
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Number.inRPM: Measure<Velocity<Angle>>
    get() = Units.RPM.of(this.toDouble())

inline val Number.inSeconds: Measure<Time>
    get() = Units.Seconds.of(this.toDouble())

inline val Number.inMinutes: Measure<Time>
    get() = Units.Minutes.of(this.toDouble())

inline val Number.inMilliseconds: Measure<Time>
    get() = Units.Milliseconds.of(this.toDouble())

inline val Number.inVolts: Measure<Voltage>
    get() = Units.Volts.of(this.toDouble())


/**
 * Converts a measure into a given unit, return the value as a double.
 *
 * @return The measure's magnitude converted into the given unit.
 * @see Measure.in
 *
 * @since 2024-02-08
 */
infix fun <U : Unit<U>> Measure<U>.into(unit: U): Double {
    return this.`in`(unit)
}
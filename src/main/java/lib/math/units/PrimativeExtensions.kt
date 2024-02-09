/**
 * Copyright (c) 2024 Matthew Clark and FRC Robotics Team 2537 "Space Raiders"
 * All rights reserved
 *
 * - - -
 *
 * A simple extension set for the [edu.wpi.first.units] library.
 *
 * This extension set aims to reduce the boilerplate and verbosity of the
 * wpilib units library while maintaining its inherent type-safety and unit
 * conversion. The extension is built for the Kotlin programming language,
 * making heavy use of Kotlin's extension function syntax, which may not
 * work as intended in Java.
 *
 * A thank you to the other members of Team 2537 "Space Raiders" for helping avoid
 * naming conflicts while maintaining clarity.
 *
 * @author Matthew Clark
 *
 * @since 2024-02-09
 */

package lib.math.units

import edu.wpi.first.units.Units

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

inline val Double.seconds: TimeSpan
    get() = Units.Seconds.of(this)

inline val Double.minutes: TimeSpan
    get() = Units.Minutes.of(this)

inline val Double.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this)

inline val Float.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Float.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Float.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Float.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Float.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Float.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Float.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Float.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Float.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Float.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Float.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Float.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Float.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Float.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Float.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Float.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Float.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())

inline val Int.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Int.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Int.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Int.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Int.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Int.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Int.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Int.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Int.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Int.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Int.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Int.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Int.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Int.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Int.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Int.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Int.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())

inline val Long.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Long.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Long.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Long.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Long.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Long.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Long.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Long.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Long.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Long.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Long.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Long.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Long.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Long.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Long.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Long.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Short.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Short.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Short.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Short.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Short.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Short.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Short.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Short.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Short.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Short.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Short.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Short.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Short.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Short.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Short.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Short.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Short.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())

inline val Long.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())

inline val Byte.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Byte.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Byte.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Byte.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Byte.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Byte.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Byte.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Byte.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Byte.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Byte.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Byte.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Byte.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Byte.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Byte.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Byte.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Byte.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Byte.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())

inline val Number.meters: Span
    get() = Units.Meters.of(this.toDouble())

inline val Number.feet: Span
    get() = Units.Feet.of(this.toDouble())

inline val Number.centimeters: Span
    get() = Units.Centimeters.of(this.toDouble())

inline val Number.millimeters: Span
    get() = Units.Millimeters.of(this.toDouble())

inline val Number.radians: Rotation
    get() = Units.Radians.of(this.toDouble())

inline val Number.degrees: Rotation
    get() = Units.Degrees.of(this.toDouble())

inline val Number.rotations: Rotation
    get() = Units.Rotations.of(this.toDouble())

inline val Number.revolutions: Rotation
    get() = Units.Revolutions.of(this.toDouble())

inline val Number.metersPerSecond: SpanVelocity
    get() = Units.MetersPerSecond.of(this.toDouble())

inline val Number.feetPerSecond: SpanVelocity
    get() = Units.FeetPerSecond.of(this.toDouble())

inline val Number.inchesPerSecond: SpanVelocity
    get() = Units.InchesPerSecond.of(this.toDouble())

inline val Number.degreesPerSecond: RotationVelocity
    get() = Units.DegreesPerSecond.of(this.toDouble())

inline val Number.radiansPerSecond: RotationVelocity
    get() = Units.RadiansPerSecond.of(this.toDouble())

inline val Number.rpm: RotationVelocity
    get() = Units.RPM.of(this.toDouble())

inline val Number.seconds: TimeSpan
    get() = Units.Seconds.of(this.toDouble())

inline val Number.minutes: TimeSpan
    get() = Units.Minutes.of(this.toDouble())

inline val Number.milliseconds: TimeSpan
    get() = Units.Milliseconds.of(this.toDouble())
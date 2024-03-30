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
 * @since 2024-02-08
 */

package lib.math.units

import com.revrobotics.CANSparkBase
import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.*
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units.*


// TODO: (I will) document all this later; I'm kinda lazy right now
infix fun <U : Unit<U>, D : Unit<D>> Unit<U>.per(denominator: D): Per<U, D> {
    @Suppress("UNCHECKED_CAST")
    return Per.combine(this as U, denominator)
}

infix fun <U : Unit<U>> Measure<U>.per(period: Measure<Time>): Measure<Velocity<U>> {
    return unit() per period.unit() outof (this.magnitude / period.magnitude)
}

infix fun <U : Unit<U>> Measure<U>.per(period: Time): Measure<Velocity<U>> {
    return unit() per period outof this.magnitude
}

infix fun <U : Unit<U>> Unit<U>.per(period: Time): Velocity<U> {
    return this.per(period)
}

operator fun <U : Unit<U>> Measure<U>.div(measure: Measure<U>): Double {
    return this.magnitude / (measure into this.unit)
}

operator fun <U : Unit<U>, V : Unit<V>> Measure<U>.div(measure: Measure<V>): Measure<Per<U, V>> {
    return unit per measure.unit outof (this.magnitude / measure.magnitude)
}

infix fun <U : Unit<U>, V : Unit<V>> Measure<U>.per(unit: V): Measure<Per<U, V>> {
    return this.per(unit)
}

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

/**
 * Gets the measure out of a unit and magnitude.
 *
 * @return A measure of the unit with given magnitude.
 * @see Unit.of
 *
 * @since 2024-03-29
 */
@Suppress("SpellCheckingInspection")
infix fun <U : Unit<U>> Unit<U>.outof(magnitude: Double): Measure<U> {
    return this.of(magnitude)
}

/**
 * The unit of the measure.
 *
 * @return The unit of this measure
 * @see Measure.unit
 *
 * @since 2024-03-29
 */
inline val <U : Unit<U>> Measure<U>.unit: U
    get() = this.unit()

/**
 * The magnitude of the measure.
 *
 * @return The magnitude of this measure
 * @see Measure.magnitude
 *
 * @since 2024-03-29
 */
inline val Measure<*>.magnitude: Double
    get() = this.magnitude()

/**
 * The base unit magnitude of the measure.
 *
 * @return The magnitude of this measure in its  base unit
 * @see Measure.baseUnitMagnitude
 *
 * @since 2024-03-29
 */
inline val Measure<*>.baseUnitMagnitude: Double
    get() = this.baseUnitMagnitude()

/**
 * Negates a [Measure] through its [Measure.negate] method
 *
 * @return The resulting measure
 * @see Measure.negate
 *
 * @since 2024-02-08
 */
operator fun <U : Unit<U>?> Measure<U>.unaryMinus(): Measure<U> {
    return this.negate()
}

/**
 * Does nothing to a measure. If this measure was mutable, it will return a mutable copy
 *
 * @return The resulting measure
 *
 * @since 2024-02-08
 */
operator fun <U : Unit<U>?> Measure<U>.unaryPlus(): Measure<U> {
    return if(this is ImmutableMeasure) this else mutableCopy()
}

/**
 * The velocity that the motor is rotating at. The velocity is measured by
 * this motor's [com.revrobotics.RelativeEncoder], and controlled by its
 * [com.revrobotics.SparkPIDController].
 *
 * @see CANSparkBase.encoder
 * @see CANSparkBase.pidController
 *
 * @since 2024-02-08
 */
var CANSparkBase.velocity: RotationVelocity
    get() = encoder.velocity.rpm
    set(value) {
        pidController.setReference(value into RPM, CANSparkBase.ControlType.kVelocity)
    }

typealias RotationVelocity = Measure<Velocity<Angle>>
typealias SpanVelocity = Measure<Velocity<Distance>>
typealias Speed = SpanVelocity
typealias Span = Measure<Distance>
typealias Rotation = Measure<Angle>
typealias TimeSpan = Measure<Time>

operator fun <U : Unit<U>> Measure<U>.rangeTo(endInclusive: Measure<U>): ClosedMeasureRange<Measure<U>, U> {
    return ClosedMeasureRange(this, endInclusive);
}

operator fun <U : Unit<U>> Measure<U>.rangeUntil(endExclusive: Measure<U>): OpenEndMeasureRange<Measure<U>, U> {
    return OpenEndMeasureRange(this, endExclusive);
}

infix fun <U : Unit<U>> Measure<U>.downTo(endInclusive: Measure<U>): UnSteppedMeasureProgression<U> {
    return UnSteppedMeasureProgression(this, endInclusive)
}

class UnSteppedMeasureProgression<U : Unit<U>> (val start: Measure<U>, val endInclusive: Measure<U>) {
    infix fun step(step: Double): ClosedMeasureProgression<U> {
        return ClosedMeasureProgression(start, endInclusive, step)
    }
}

class ClosedMeasureRange<M, U>(override val start: M, override val endInclusive: M) : ClosedFloatingPointRange<M>
    where M : Measure<U>, M : Comparable<Measure<U>>, U : Unit<U>
{
    override fun lessThanOrEquals(a: M, b: M): Boolean = a <= b

    @Suppress("ConvertTwoComparisonsToRangeCheck")
    override fun contains(value: M): Boolean = start <= value && value <= endInclusive
    override fun isEmpty(): Boolean = start > endInclusive

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as ClosedMeasureRange<*, *>

        if (endInclusive != other.endInclusive) return false
        if (start != other.start) return false

        return true
    }

    override fun hashCode(): Int {
        var result = endInclusive.hashCode()
        result = 31 * result + start.hashCode()
        return result
    }

    override fun toString(): String {
        return "$endInclusive..$start"
    }

    infix fun step(step: Double): ClosedMeasureProgression<U> {
        return ClosedMeasureProgression(this.start, this.endInclusive, step)
    }

}

open class ClosedMeasureProgression<U : Unit<U>>(val start: Measure<U>, val endInclusive: Measure<U>, val step: Double)
    : Iterable<Measure<U>> {
    /**
     * Returns an iterator over the elements of this object.
     */
    override fun iterator(): Iterator<Measure<U>> {
        return object : Iterator<Measure<U>> {
            var current: MutableMeasure<U> = MutableMeasure.mutable(start)
            val step: Measure<U> = current.unit().of(this@ClosedMeasureProgression.step)

            override fun hasNext(): Boolean {
                // Prevent garbage measures from being created
                val ret = current.mut_plus(step) <= this@ClosedMeasureProgression.endInclusive
                current.mut_minus(step)
                return ret
            }

            override fun next(): Measure<U> {
                return current.mut_plus(step)
            }

        }
    }

}

class OpenEndMeasureRange<M, U>(override val start: M, override val endExclusive: M) : OpenEndRange<M>
    where M : Measure<U>, M : Comparable<Measure<U>>, U : Unit<U>
{
    @Suppress("ConvertTwoComparisonsToRangeCheck")
    override fun contains(value: M): Boolean = start <= value && value < endExclusive

    override fun isEmpty(): Boolean = start >= endExclusive

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as OpenEndMeasureRange<*, *>

        if (start != other.start) return false
        if (endExclusive != other.endExclusive) return false

        return true
    }

    override fun hashCode(): Int {
        var result = start.hashCode()
        result = 31 * result + endExclusive.hashCode()
        return result
    }

    override fun toString(): String {
        return "$start..<$endExclusive"
    }

}


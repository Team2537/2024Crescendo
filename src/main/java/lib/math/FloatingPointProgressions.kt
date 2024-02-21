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


package lib.math

interface FloatingPointProgression<T : Comparable<T>> : Iterable<T> {
    val start: T
    val end: T
    val step: T

    val isAscending: Boolean

    fun isEmpty(): Boolean = if (isAscending) start > end else start < end
}

// TODO: This for floats
class DoubleProgression(override val start: Double, override val end: Double, override val step: Double) : FloatingPointProgression<Double> {
    override val isAscending: Boolean = step > 0.0

    override fun iterator(): DoubleIterator {
        return object : DoubleIterator() {
            var current: Double = start

            override fun hasNext(): Boolean = current + step < end

            override fun nextDouble(): Double {
                current += step
                return current
            }
        }
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as DoubleProgression

        if (start != other.start) return false
        if (end != other.end) return false
        if (step != other.step) return false

        return true
    }

    override fun hashCode(): Int {
        var result = start.hashCode()
        result = 31 * result + end.hashCode()
        result = 31 * result + step.hashCode()
        return result
    }

    override fun toString(): String {
        return if (isAscending) "$start..$end step $step" else "$start downTo $end step $step"
    }
}

infix fun ClosedFloatingPointRange<Double>.step(step: Double): DoubleProgression
    = DoubleProgression(this.start, this.endInclusive, step)
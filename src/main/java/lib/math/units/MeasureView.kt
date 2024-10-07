package lib.math.units

import edu.wpi.first.units.*
import edu.wpi.first.units.Unit

/**
 * A read-only view into a potentially mutable measure.
 *
 * Returning [mutable measures][MutableMeasure] as [Measure]
 * types does some surface-level protection of mutability, but does not
 * protect fully, as one can still cast the return value and access any
 * mutation method.
 *
 * This class circumvents this by providing delegate methods for only
 * read operations.
 *
 * @param U The type of unit of the viewed measure.
 *
 * @author Matthew Clark
 *
 * @see Measure
 * @see MutableMeasure
 *
 * @constructor Constructs a view into the given measure, unable to mutate it.
 *
 * @param view The measure to view.
 */
class MeasureView<U : Unit<U>>(private val view: Measure<U>) : Measure<U> {
    /**
     * Gets the unit-less magnitude of this measure.
     *
     * @return the magnitude in terms of [the unit][unit].
     */
    override fun magnitude(): Double {
        return view.magnitude()
    }

    /**
     * Gets the magnitude of this measure in terms of the base unit. If the unit is the base unit for
     * its system of measure, then the value will be equivalent to [magnitude].
     *
     * @return the magnitude in terms of the base unit
     */
    override fun baseUnitMagnitude(): Double {
        return view.baseUnitMagnitude()
    }

    /**
     * Gets the units of this measure.
     *
     * @return the unit
     */
    override fun unit(): U {
        return view.unit()
    }

    /**
     * Converts this measure to a measure with a different unit of the same type, e.g. minutes to
     * seconds. Converting to the same unit is equivalent to calling [magnitude].
     *
     * ```Java
     * Meters.of(12).in(Feet) // 39.3701
     * Seconds.of(15).in(Minutes) // 0.25
     * ```
     *
     * @param unit the unit to convert this measure to
     * @return the value of this measure in the given unit
     */
    override fun `in`(unit: Unit<U>): Double {
        return view.`in`(unit)
    }

    /**
     * Multiplies this measurement by some constant multiplier and returns the result. The magnitude
     * of the result will be the *base* magnitude multiplied by the scalar value. If the measure
     * uses a unit with a non-linear relation to its base unit (such as Fahrenheit for temperature),
     * then the result will only be a multiple *in terms of the base unit*.
     *
     * @param multiplier the constant to multiply by
     * @return the resulting measure
     */
    override operator fun times(multiplier: Double): Measure<U> {
        return view.times(multiplier)
    }

    /**
     * Generates a new measure that is equal to this measure multiplied by another. Some dimensional
     * analysis is performed to reduce the units down somewhat; for example, multiplying a `Measure<Time>` by a
     * `Measure<Velocity<Distance>>` will return just a `Measure<Distance>` instead of the naive
     * `Measure<Mult<Time, Velocity<Distance>>`. This is not guaranteed to perform perfect dimensional analysis.
     *
     * @param other the unit to multiply by
     * @return the multiplicative unit
     */
    override operator fun <U2 : Unit<U2>> times(other: Measure<U2>): Measure<*> {
        return view.times(other)
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * `times(1 / divisor)`
     *
     * @param divisor the constant to divide by
     * @return the resulting measure
     * @see times
     */
    override fun divide(divisor: Double): Measure<U> {
        return view.divide(divisor)
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * `times(1 / divisor)`
     *
     * @param divisor the constant to divide by
     * @return the resulting measure
     * @see times
     */
    operator fun div(divisor: Double): Measure<U> {
        return divide(divisor) // operator wrapper
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * `divide(divisor.baseUnitMagnitude())`
     *
     * @param divisor the dimensionless measure to divide by
     * @return the resulting measure
     * @see .divide
     * @see .times
     */
    override fun divide(divisor: Measure<Dimensionless>): Measure<U> {
        return view.divide(divisor)
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * `divide(divisor.baseUnitMagnitude())`
     *
     * @param divisor the dimensionless measure to divide by
     * @return the resulting measure
     * @see .divide
     * @see .times
     */
    operator fun div(divisor: Measure<Dimensionless>): Measure<U> {
        return divide(divisor)
    }

    /**
     * Creates a velocity measure by dividing this one by a time period measure.
     *
     * ```Java
     * Meters.of(1).per(Second) // Measure<Velocity<Distance>>
     * ```
     *
     * @param period the time period to divide by.
     * @return the velocity result
     */
    override fun per(period: Measure<Time>): Measure<Velocity<U>> {
        return view.per(period)
    }

    /**
     * Creates a relational measure equivalent to this one per some other unit.
     *
     * ```Java
     * Volts.of(1.05).per(Meter) // V/m, potential PID constant
     * ```
     *
     * @param denominator the denominator unit being divided by
     * @return the relational measure
     */
    override fun <U2 : Unit<U2>> per(denominator: U2): Measure<Per<U, U2>> {
        return view.per(denominator)
    }

    /**
     * Creates a velocity measure equivalent to this one per a unit of time.
     *
     * ```Java
     * Radians.of(3.14).per(Second) // Velocity<Angle> equivalent to RadiansPerSecond.of(3.14)
     * ```
     *
     * @param time the unit of time
     * @return the velocity measure
     */
    override fun per(time: Time): Measure<Velocity<U>> {
        return view.per(time)
    }

    /**
     * Adds another measure to this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to add to this one
     * @return a new measure containing the result
     */
    override operator fun plus(other: Measure<U>): Measure<U> {
        return view.plus(other)
    }

    /**
     * Subtracts another measure from this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to subtract from this one
     * @return a new measure containing the result
     */
    override operator fun minus(other: Measure<U>): Measure<U> {
        return view.minus(other)
    }

    /**
     * Negates this measure and returns the result.
     *
     * @return the resulting measure
     */
    override fun negate(): Measure<U> {
        return view.negate()
    }

    /**
     * Returns an immutable copy of this measure. The copy can be used freely and is guaranteed never
     * to change.
     *
     * @return the copied measure
     */
    override fun copy(): Measure<U> {
        return view.copy()
    }

    /**
     * Creates a new mutable copy of this measure.
     *
     * @return a mutable measure initialized to be identical to this measure
     */
    override fun mutableCopy(): MutableMeasure<U> {
        return view.mutableCopy()
    }

    /**
     * Checks if this measure is near another measure of the same unit. Provide a variance threshold
     * for use for a +/- scalar, such as 0.05 for +/- 5%.
     *
     * ```Java
     * Inches.of(11).isNear(Inches.of(10), 0.1) // true
     * Inches.of(12).isNear(Inches.of(10), 0.1) // false
     * ```
     *
     * @param other             the other measurement to compare against
     * @param varianceThreshold the acceptable variance threshold, in terms of an acceptable +/- error
     * range multiplier. Checking if a value is within 10% means a value of 0.1 should be passed;
     * checking if a value is within 1% means a value of 0.01 should be passed, and so on.
     * @return true if this unit is near the other measure, otherwise false
     */
    override fun isNear(other: Measure<*>?, varianceThreshold: Double): Boolean {
        return view.isNear(other, varianceThreshold)
    }

    /**
     * Checks if this measure is equivalent to another measure of the same unit.
     *
     * @param other the measure to compare to
     * @return true if this measure is equivalent, false otherwise
     */
    override fun isEquivalent(other: Measure<*>?): Boolean {
        return view.isEquivalent(other)
    }

    /**
     * Compares this object with the specified object for order. Returns a negative integer, zero, or a positive integer
     * as this object is less than, equal to, or greater than the specified object.
     *
     * The implementor must ensure `signum(x.compareTo(y)) == -signum(y.compareTo(x))` for all x and y. (This implies
     * that `x.compareTo(y)` must throw an exception if and only if `y.compareTo(x)` throws an exception.)
     *
     * The implementor must also ensure that the relation is transitive: (`x.compareTo(y) > 0 && y.compareTo(z) > 0`)
     * implies `x.compareTo(z) > 0`.
     *
     * Finally, the implementor must ensure that `x.compareTo(y) == 0` implies that
     * `signum(x.compareTo(z)) == signum(y.compareTo(z))`, for all z.
     */
    override operator fun compareTo(other: Measure<U>): Int {
        return view.compareTo(other)
    }

    /**
     * Checks if this measure is greater than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a greater equivalent magnitude, false otherwise
     */
    override fun gt(o: Measure<U>): Boolean {
        return view.gt(o)
    }

    /**
     * Checks if this measure is greater than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or greater equivalent magnitude, false otherwise
     */
    override fun gte(o: Measure<U>): Boolean {
        return view.gte(o)
    }

    /**
     * Checks if this measure is less than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a lesser equivalent magnitude, false otherwise
     */
    override fun lt(o: Measure<U>): Boolean {
        return view.lt(o)
    }

    /**
     * Checks if this measure is less than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or lesser equivalent magnitude, false otherwise
     */
    override fun lte(o: Measure<U>): Boolean {
        return view.lte(o)
    }

    /**
     * Returns a string representation of this measurement in a shorthand form. The symbol of the
     * backing unit is used, rather than the full name, and the magnitude is represented in scientific
     * notation.
     *
     * @return the short form representation of this measurement
     */
    override fun toShortString(): String {
        return view.toShortString()
    }

    /**
     * Returns a string representation of this measurement in a longhand form. The name of the backing
     * unit is used, rather than its symbol, and the magnitude is represented in a full string, not
     * scientific notation. (Very large values may be represented in scientific notation, however)
     *
     * @return the long form representation of this measurement
     */
    override fun toLongString(): String {
        return view.toLongString()
    }

    override fun toString(): String {
        return "MeasureView{" +
                view +
                '}'
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other == null || javaClass != other.javaClass) return false
        other as MeasureView<*>
        return view == other.view
    }

    override fun hashCode(): Int {
        return view.hashCode()
    }
}

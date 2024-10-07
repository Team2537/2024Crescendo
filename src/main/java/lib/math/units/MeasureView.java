package lib.math.units;

import edu.wpi.first.units.*;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

/**
 * A read-only view into a potentially mutable measure.
 * <p>
 * Returning {@link MutableMeasure mutable measures} as {@link Measure}
 * types does some surface-level protection of mutability, but does not
 * protect fully, as one can still cast the return value and access any
 * mutation method.
 * <p>
 * This class circumvent this by providing delegate methods for only
 * read operations.
 *
 * @param <U> The type of unit of the viewed measure.
 *
 * @author Matthew Clark
 *
 * @see Measure
 * @see MutableMeasure
 */
public final class MeasureView<U extends Unit<U>> implements Measure<U> {
    @NotNull
    private final Measure<U> view;

    public MeasureView(@NotNull Measure<U> view) {
        this.view = view;
    }

    /**
     * Gets the unit-less magnitude of this measure.
     *
     * @return the magnitude in terms of {@link #unit() the unit}.
     */
    @Override
    public double magnitude() {
        return view.magnitude();
    }

    /**
     * Gets the magnitude of this measure in terms of the base unit. If the unit is the base unit for
     * its system of measure, then the value will be equivalent to {@link #magnitude()}.
     *
     * @return the magnitude in terms of the base unit
     */
    @Override
    public double baseUnitMagnitude() {
        return view.baseUnitMagnitude();
    }

    /**
     * Gets the units of this measure.
     *
     * @return the unit
     */
    @Override
    public U unit() {
        return view.unit();
    }

    /**
     * Converts this measure to a measure with a different unit of the same type, e.g. minutes to
     * seconds. Converting to the same unit is equivalent to calling {@link #magnitude()}.
     *
     * <pre>
     *   Meters.of(12).in(Feet) // 39.3701
     *   Seconds.of(15).in(Minutes) // 0.25
     * </pre>
     *
     * @param unit the unit to convert this measure to
     * @return the value of this measure in the given unit
     */
    @Override
    public double in(Unit<U> unit) {
        return view.in(unit);
    }

    /**
     * Multiplies this measurement by some constant multiplier and returns the result. The magnitude
     * of the result will be the <i>base</i> magnitude multiplied by the scalar value. If the measure
     * uses a unit with a non-linear relation to its base unit (such as Fahrenheit for temperature),
     * then the result will only be a multiple <i>in terms of the base unit</i>.
     *
     * @param multiplier the constant to multiply by
     * @return the resulting measure
     */
    @Override
    public Measure<U> times(double multiplier) {
        return view.times(multiplier);
    }

    /**
     * Generates a new measure that is equal to this measure multiplied by another. Some dimensional
     * analysis is performed to reduce the units down somewhat; for example, multiplying a {@code
     * Measure<Time>} by a {@code Measure<Velocity<Distance>>} will return just a {@code
     * Measure<Distance>} instead of the naive {@code Measure<Mult<Time, Velocity<Distance>>}. This is
     * not guaranteed to perform perfect dimensional analysis.
     *
     * @param other the unit to multiply by
     * @return the multiplicative unit
     */
    @Override
    public <U2 extends Unit<U2>> Measure<?> times(Measure<U2> other) {
        return view.times(other);
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * {@code times(1 / divisor)}
     *
     * @param divisor the constant to divide by
     * @return the resulting measure
     * @see #times(double)
     */
    @Override
    public Measure<U> divide(double divisor) {
        return view.divide(divisor);
    }

    /**
     * Divides this measurement by some constant divisor and returns the result. This is equivalent to
     * {@code divide(divisor.baseUnitMagnitude())}
     *
     * @param divisor the dimensionless measure to divide by
     * @return the resulting measure
     * @see #divide(double)
     * @see #times(double)
     */
    @Override
    public Measure<U> divide(Measure<Dimensionless> divisor) {
        return view.divide(divisor);
    }

    /**
     * Creates a velocity measure by dividing this one by a time period measure.
     *
     * <pre>
     *   Meters.of(1).per(Second) // Measure&lt;Velocity&lt;Distance&gt;&gt;
     * </pre>
     *
     * @param period the time period to divide by.
     * @return the velocity result
     */
    @Override
    public Measure<Velocity<U>> per(Measure<Time> period) {
        return view.per(period);
    }

    /**
     * Creates a relational measure equivalent to this one per some other unit.
     *
     * <pre>
     *   Volts.of(1.05).per(Meter) // V/m, potential PID constant
     * </pre>
     *
     * @param denominator the denominator unit being divided by
     * @return the relational measure
     */
    @Override
    public <U2 extends Unit<U2>> Measure<Per<U, U2>> per(U2 denominator) {
        return view.per(denominator);
    }

    /**
     * Creates a velocity measure equivalent to this one per a unit of time.
     *
     * <pre>
     *   Radians.of(3.14).per(Second) // Velocity&lt;Angle&gt; equivalent to RadiansPerSecond.of(3.14)
     * </pre>
     *
     * @param time the unit of time
     * @return the velocity measure
     */
    @Override
    public Measure<Velocity<U>> per(Time time) {
        return view.per(time);
    }

    /**
     * Adds another measure to this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to add to this one
     * @return a new measure containing the result
     */
    @Override
    public Measure<U> plus(Measure<U> other) {
        return view.plus(other);
    }

    /**
     * Subtracts another measure from this one. The resulting measure has the same unit as this one.
     *
     * @param other the measure to subtract from this one
     * @return a new measure containing the result
     */
    @Override
    public Measure<U> minus(Measure<U> other) {
        return view.minus(other);
    }

    /**
     * Negates this measure and returns the result.
     *
     * @return the resulting measure
     */
    @Override
    public Measure<U> negate() {
        return view.negate();
    }

    /**
     * Returns an immutable copy of this measure. The copy can be used freely and is guaranteed never
     * to change.
     *
     * @return the copied measure
     */
    @Override
    public Measure<U> copy() {
        return view.copy();
    }

    /**
     * Creates a new mutable copy of this measure.
     *
     * @return a mutable measure initialized to be identical to this measure
     */
    @Override
    public MutableMeasure<U> mutableCopy() {
        return view.mutableCopy();
    }

    /**
     * Checks if this measure is near another measure of the same unit. Provide a variance threshold
     * for use for a +/- scalar, such as 0.05 for +/- 5%.
     *
     * <pre>
     *   Inches.of(11).isNear(Inches.of(10), 0.1) // true
     *   Inches.of(12).isNear(Inches.of(10), 0.1) // false
     * </pre>
     *
     * @param other             the other measurement to compare against
     * @param varianceThreshold the acceptable variance threshold, in terms of an acceptable +/- error
     *                          range multiplier. Checking if a value is within 10% means a value of 0.1 should be passed;
     *                          checking if a value is within 1% means a value of 0.01 should be passed, and so on.
     * @return true if this unit is near the other measure, otherwise false
     */
    @Override
    public boolean isNear(Measure<?> other, double varianceThreshold) {
        return view.isNear(other, varianceThreshold);
    }

    /**
     * Checks if this measure is equivalent to another measure of the same unit.
     *
     * @param other the measure to compare to
     * @return true if this measure is equivalent, false otherwise
     */
    @Override
    public boolean isEquivalent(Measure<?> other) {
        return view.isEquivalent(other);
    }

    /**
     * {@inheritDoc}
     *
     * @param o
     */
    @Override
    public int compareTo(Measure<U> o) {
        return view.compareTo(o);
    }

    /**
     * Checks if this measure is greater than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a greater equivalent magnitude, false otherwise
     */
    @Override
    public boolean gt(Measure<U> o) {
        return view.gt(o);
    }

    /**
     * Checks if this measure is greater than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or greater equivalent magnitude, false otherwise
     */
    @Override
    public boolean gte(Measure<U> o) {
        return view.gte(o);
    }

    /**
     * Checks if this measure is less than another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has a lesser equivalent magnitude, false otherwise
     */
    @Override
    public boolean lt(Measure<U> o) {
        return view.lt(o);
    }

    /**
     * Checks if this measure is less than or equivalent to another measure of the same unit.
     *
     * @param o the other measure to compare to
     * @return true if this measure has an equal or lesser equivalent magnitude, false otherwise
     */
    @Override
    public boolean lte(Measure<U> o) {
        return view.lte(o);
    }

    /**
     * Returns a string representation of this measurement in a shorthand form. The symbol of the
     * backing unit is used, rather than the full name, and the magnitude is represented in scientific
     * notation.
     *
     * @return the short form representation of this measurement
     */
    @Override
    public String toShortString() {
        return view.toShortString();
    }

    /**
     * Returns a string representation of this measurement in a longhand form. The name of the backing
     * unit is used, rather than its symbol, and the magnitude is represented in a full string, not
     * scientific notation. (Very large values may be represented in scientific notation, however)
     *
     * @return the long form representation of this measurement
     */
    @Override
    public String toLongString() {
        return view.toLongString();
    }

    @Override
    public String toString() {
        return "MeasureView{" +
                view +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        MeasureView<?> that = (MeasureView<?>) o;
        return Objects.equals(view, that.view);
    }

    @Override
    public int hashCode() {
        return view.hashCode();
    }
}

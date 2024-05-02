package lib.math

import kotlin.math.absoluteValue
import kotlin.math.pow

/**
 * Raises a base to an exponent *specifically to scale its value.*
 *
 * It will retain its sign, and will give proper (but negative) values
 * for all negative bases for all fractional exponents
 *
 * @param exp The exponent to raise to
 *
 * @return The resulting double, with sign preserved. If neither parameters
 * are [NaN][Double.NaN], the result will never be NaN.
 *
 * @author Matthew Clark
 */
fun Double.powScale(exp: Double): Double {
    // (-1.0) ** 2.0 > 0                        <- bad
    // (-1.0) * |-1.0| ** (2.0 - 1) < 0         <- good
    // (-1.0) ** (3.0 / 2.0) => NaN             <- bad
    // (-1.0) * |-1.0| ** (2.0 / 3.0 - 1) == 1  <- good
    return this * this.absoluteValue.pow(exp - 1.0)
}

/**
 * Raises a base to an exponent *specifically to scale its value.*
 *
 * It will retain its sign, and will give proper (but negative) values
 * for all negative bases for all fractional exponents
 *
 * @param exp The exponent to raise to
 *
 * @return The resulting float, with sign preserved. If neither parameters
 * are [NaN][Float.NaN], the result will never be NaN.
 *
 * @author Matthew Clark
 */
fun Float.powScale(exp: Float): Float {
    // (-1.0) ** 2.0 > 0                        <- bad
    // (-1.0) * |-1.0| ** (2.0 - 1) < 0         <- good
    // (-1.0) ** (3.0 / 2.0) => NaN             <- bad
    // (-1.0) * |-1.0| ** (2.0 / 3.0 - 1) == 1  <- good
    return this * this.absoluteValue.pow(exp - 1.0f)
}

/**
 * Raises a base to an exponent *specifically to scale its value.*
 *
 * It will retain its sign, and will give proper (but negative) values
 * for all negative bases for all fractional exponents
 *
 * @param exp The exponent to raise to
 *
 * @return The resulting number, calculated by getting both numbers'
 * [Number.toDouble] method.
 *
 * @author Matthew Clark
 */
fun Number.powScale(exp: Number): Number {
    return this.toDouble().powScale(exp.toDouble())
}

val slowmodeFunction: (Double) -> Double = { x: Double -> (-0.8 * x) + 1.0 }


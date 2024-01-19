package lib

import edu.wpi.first.wpilibj2.command.button.Trigger
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
fun Boolean.toTrigger(): Trigger {
    return Trigger { this }
}

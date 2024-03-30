
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
 */
package lib.math.units

import com.ctre.phoenix6.BaseStatusSignal
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units

inline val BaseStatusSignal.measure: Measure<*>
    get() {
        val unit = this.units // FIXME: No idea what sanitation this is gonna need

        Units::class.java.declaredFields.forEach {
            if(unit == it.name){
                return it.get(null) as Measure<*>
            }
        }

        // TODO: log a non-fatal error here probably
//        throw IllegalArgumentException("Could not find existing unit of name $unit")
        return Units.AnonymousBaseUnit.of(this.valueAsDouble)
    }

infix fun <U : Unit<U>> BaseStatusSignal.measure(unit: U): Measure<U> {
    return unit outof this.valueAsDouble
}

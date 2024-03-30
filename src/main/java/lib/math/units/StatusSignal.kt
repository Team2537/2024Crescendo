
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
import java.util.*
import kotlin.collections.ArrayList

// Potential to allow at-runtime addition of unit
private var unitsToSearch: ArrayList<Unit<*>>? = null

/**
 * Gets a measure of this status signal
 *
 * The measure will be in the unit type found in [BaseStatusSignal.getUnits] if
 * said unit type can be found in the [Units] class. This method will reflectively
 * search the Units class for a unit with a matching name to this signal's units.
 *
 * If no proper unit can be found, a unit-less measure with this signal's magnitude
 * will be returned.
 *
 * The unit type of this measure **cannot** be found at compile time.
 *
 * @return The [Measure] of this signal.
 *
 * @since 2024-03-29
 */
val BaseStatusSignal.measure: Measure<*>
    get() {
        val unit = this.units // FIXME: No idea what sanitation this is gonna need

        // Populate the list of units to search once
        // using a list to make it possible to add units in the future
        if(unitsToSearch == null){
            val arr = Units::class.java.declaredFields
            unitsToSearch = ArrayList(arr.size)
            arr.forEach {
                // nvm figured it out
                unitsToSearch!!.add(it.get(null) as Unit<*>)
            }
        } else {
            unitsToSearch!!.forEach {
                if(unit == it.name()){
                    return it.of(this.valueAsDouble)
                }
            }
        }


        // TODO: log a non-fatal error here probably
//        throw IllegalArgumentException("Could not find existing unit of name $unit")
        return Units.AnonymousBaseUnit.of(this.valueAsDouble)
    }

/**
 * Gets a measure of this status signal with a given unit.
 *
 * This method will not do any checking for if the provided unit makes
 * sense; it will simply blindly create a measure with the unit and
 * this signal's magnitude.
 *
 * @param unit The unit to measure in.
 *
 * @return The [Measure] of this signal.
 *
 * @since 2024-03-29
 */
infix fun <U : Unit<U>> BaseStatusSignal.measure(unit: U): Measure<U> {
    return unit outof this.valueAsDouble
}

package frc.robot.subsystems.pivot

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface PivotIO {
    class PivotInputs : LoggableInputs {
        /** Whether the pivot arm is at its hardstop. */
        @JvmField
        var isAtHardstop: Boolean = false

        /** The position of the motor(s), measured relatively by the motor(s)'s encoder. */
        @JvmField
        val relativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The position of the motor(s), measured absolutely. */
        @JvmField
        val absolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The velocity of the motor(s)'s rotation. */
        @JvmField
        val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

        /** The voltage applied to the motor(s). */
        @JvmField
        val appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** The current applied to the motor(s). */
        @JvmField
        val appliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)


        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("isAtHardstop", isAtHardstop)

            table.put("relativePosition", relativePosition)
            table.put("absolutePosition", absolutePosition)
            table.put("appliedVoltage", appliedVoltage)
            table.put("appliedCurrent", appliedCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            table.get("isAtHardstop").let { isAtHardstop = it.boolean }

            relativePosition.mut_replace(table.get("relativePosition", relativePosition))
            absolutePosition.mut_replace(table.get("absolutePosition", absolutePosition))
            appliedVoltage.mut_replace(table.get("appliedVoltage", appliedVoltage))
            appliedCurrent.mut_replace(table.get("appliedCurrent", appliedCurrent))
        }

    }

    fun updateInputs(inputs: PivotInputs) {}

    fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean = false) {}

    fun setKnownPosition(position: Measure<Angle>) {}

    fun setTargetPosition(position: Measure<Angle>) {}

    fun setPID(p: Double, i: Double, d: Double) {}

    fun setFF(kS: Double, kG: Double, kV: Double, kA: Double) {}
}
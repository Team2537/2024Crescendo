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
        var isAtHardstop: Boolean = false

        /** The position of the motor(s), measured relatively by the motor(s)'s encoder. */
        val relativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The position of the motor(s), measured absolutely. */
        val absolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The velocity of the motor(s)'s rotation. */
        val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

        /** The voltage applied to the motor(s). */
        val appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** The current applied to the motor(s). */
        val appliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)


        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("isAtHardstop", isAtHardstop)

            table.put("pivotRelativePosition", relativePosition)
            table.put("pivotAbsolutePosition", absolutePosition)
            table.put("pivotAppliedVoltage", appliedVoltage)
            table.put("pivotAppliedCurrent", appliedCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            table.get("isAtHardstop").let { isAtHardstop = it.boolean }

            relativePosition.mut_replace(table.get("pivotRelativePosition", relativePosition))
            absolutePosition.mut_replace(table.get("pivotAbsolutePosition", absolutePosition))
            appliedVoltage.mut_replace(table.get("pivotAppliedVoltage", appliedVoltage))
            appliedCurrent.mut_replace(table.get("pivotAppliedCurrent", appliedCurrent))
        }

    }

    /**
     * Updates the inputs with new data.
     * @param inputs The data to update with the new sensor information, mutated in place.
     */
    fun updateInputs(inputs: PivotInputs) {}

    /**
     * Sets the raw voltage applied to the motor(s).
     * @param voltage The voltage to apply.
     * @param isPID Whether the voltage is being set by a PID controller. (Used for simulation purposes.)
     */
    fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean = false) {}

    /**
     * Reset the relative encoder to a known position.
     * @param position The known position to reset to.
     */
    fun setKnownPosition(position: Measure<Angle>) {}

    /**
     * Set the target position of the pivot.
     * @param position The target position to set.
     */
    fun setTargetPosition(position: Measure<Angle>) {}

    /**
     * Set the PID gains of the pivot.
     * @param p The proportional gain.
     * @param i The integral gain.
     * @param d The derivative gain.
     */
    fun setPID(p: Double, i: Double, d: Double) {}

    /**
     * Set the feedforward gains of the pivot.
     * @param kS The static gain.
     * @param kG The gravity gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    fun setFF(kS: Double, kG: Double, kV: Double, kA: Double) {}
}
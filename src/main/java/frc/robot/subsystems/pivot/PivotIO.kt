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
        var isAtHardstop: Boolean = false

        var pivotRelativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)
            private set
        var pivotAbsolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)
            private set
        var pivotAngularVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)
            private set
        var pivotMotorAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
            private set
        var pivotMotorAppliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
            private set


        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("isAtHardstop", isAtHardstop)

            table.put("pivotRelativePosition", pivotRelativePosition)
            table.put("pivotAbsolutePosition", pivotAbsolutePosition)
            table.put("pivotMotorAppliedVoltage", pivotMotorAppliedVoltage)
            table.put("pivotMotorAppliedCurrent", pivotMotorAppliedCurrent)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            table.get("isAtHardstop").let { isAtHardstop = it.boolean }

            pivotRelativePosition.mut_replace(table.get("pivotRelativePosition", pivotRelativePosition))
            pivotAbsolutePosition.mut_replace(table.get("pivotAbsolutePosition", pivotAbsolutePosition))
            pivotMotorAppliedVoltage.mut_replace(table.get("pivotMotorAppliedVoltage", pivotMotorAppliedVoltage))
            pivotMotorAppliedCurrent.mut_replace(table.get("pivotMotorAppliedCurrent", pivotMotorAppliedCurrent))
        }

    }

    fun updateInputs(inputs: PivotInputs) {}

    fun setRawVoltage(voltage: Measure<Voltage>) {}

    fun setKnownPosition(position: Measure<Angle>) {}

    fun setTargetPosition(position: Measure<Angle>) {}

    fun setPID(p: Double, i: Double, d: Double) {}

    fun setFF(kS: Double, kG: Double, kV: Double, kA: Double) {}
}
package frc.robot.subsystems.superstructure.launcher.roller

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Current
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import lib.math.units.into
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface RollerIO {
    class RollerInputs: LoggableInputs {
        val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RPM)
        val position: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)
        val statorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        val motorVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
        val supplyVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
        var noteDetected: Boolean = false

        override fun toLog(table: LogTable) {
            table.put("velocity", velocity)
            table.put("position", position)
            table.put("statorCurrent", statorCurrent)
            table.put("motorVoltage", motorVoltage)
            table.put("supplyCurrent", supplyVoltage)
            table.put("noteDetected", noteDetected)
        }

        override fun fromLog(table: LogTable) {
            velocity.mut_replace(table.get("velocity", velocity))
            position.mut_replace(table.get("position", position))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
            motorVoltage.mut_replace(table.get("motorVoltage", motorVoltage))
            supplyVoltage.mut_replace(table.get("supplyCurrent", supplyVoltage))
            noteDetected = table.get("noteDetected", noteDetected)
        }
    }

    fun updateInputs(inputs: RollerInputs) {}

    fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean = false) {}
    fun setTargetPosition(position: Measure<Distance>) {}
    fun setBrakeMode(isBrakeMode: Boolean) {}
}
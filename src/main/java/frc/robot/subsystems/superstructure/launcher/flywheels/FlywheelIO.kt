package frc.robot.subsystems.superstructure.launcher.flywheels

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface FlywheelIO {
    class FlywheelInputs : LoggableInputs {
        val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(DegreesPerSecond)
        val position: MutableMeasure<Angle> = MutableMeasure.zero(Degrees)
        val statorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)
        val motorVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
        val supplyVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        override fun toLog(table: LogTable) {
            table.put("velocity", velocity)
            table.put("position", position)
            table.put("statorCurrent", statorCurrent)
            table.put("motorVoltage", motorVoltage)
            table.put("supplyCurrent", supplyVoltage)
        }

        override fun fromLog(table: LogTable) {
            velocity.mut_replace(table.get("velocity", velocity))
            position.mut_replace(table.get("position", position))
            statorCurrent.mut_replace(table.get("statorCurrent", statorCurrent))
            motorVoltage.mut_replace(table.get("motorVoltage", motorVoltage))
            supplyVoltage.mut_replace(table.get("supplyCurrent", supplyVoltage))
        }
    }

    fun updateInputs(inputs: FlywheelInputs) {}

    fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean = false) {}
    fun setVelocity(velocity: Measure<Velocity<Angle>>) {}
    fun setBrakeMode(isBrakeMode: Boolean) {}
}
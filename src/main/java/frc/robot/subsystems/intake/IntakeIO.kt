package frc.robot.subsystems.intake

import edu.wpi.first.units.Current
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var linearVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
        var supplyVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var motorVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var statorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)
        var exitSensorTriggered: Boolean = false
        var intakeSensorTriggered: Boolean = false

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("linearVelocity", linearVelocity)
            table.put("supplyVoltage", supplyVoltage)
            table.put("motorVoltage", motorVoltage)
            table.put("statorCurrent", statorCurrent)
            table.put("exitSensorTriggered", exitSensorTriggered)
            table.put("intakeSensorTriggered", intakeSensorTriggered)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            linearVelocity = table.get("linearVelocity", linearVelocity)
            supplyVoltage = table.get("supplyVoltage", supplyVoltage)
            motorVoltage = table.get("motorVoltage", motorVoltage)
            statorCurrent = table.get("statorCurrent", statorCurrent)
            exitSensorTriggered = table.get("exitSensorTriggered", exitSensorTriggered)
            intakeSensorTriggered = table.get("intakeSensorTriggered", intakeSensorTriggered)
        }
    }
    fun updateInputs(inputs: IntakeInputs) {}
    fun setVoltage(voltage: Measure<Voltage>) {}
    fun stop() {}
}
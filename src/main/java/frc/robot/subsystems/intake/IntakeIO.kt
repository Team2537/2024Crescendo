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
        var intakeLinearVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
        var intakeSupplyVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var intakeMotorVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var intakeStatorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)
        var exitSensorTriggered: Boolean = false
        var intakeSensorTriggered: Boolean = false

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("intakeLinearVelocity", intakeLinearVelocity)
            table.put("intakeSupplyVoltage", intakeSupplyVoltage)
            table.put("intakeMotorVoltage", intakeMotorVoltage)
            table.put("intakeStatorCurrent", intakeStatorCurrent)
            table.put("exitSensorTriggered", exitSensorTriggered)
            table.put("intakeSensorTriggered", intakeSensorTriggered)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            intakeLinearVelocity = table.get("intakeLinearVelocity", intakeLinearVelocity)
            intakeSupplyVoltage = table.get("intakeSupplyVoltage", intakeSupplyVoltage)
            intakeMotorVoltage = table.get("intakeMotorVoltage", intakeMotorVoltage)
            intakeStatorCurrent = table.get("intakeStatorCurrent", intakeStatorCurrent)
            exitSensorTriggered = table.get("exitSensorTriggered", exitSensorTriggered)
            intakeSensorTriggered = table.get("intakeSensorTriggered", intakeSensorTriggered)
        }
    }
    fun updateInputs(inputs: IntakeInputs) {}
    fun setVoltage(voltage: Measure<Voltage>) {}
    fun stop() {}
}
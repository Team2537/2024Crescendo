package frc.robot.subsystems.intake

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeInputs : LoggableInputs {
        var intakeLinearVelocity: Double = 0.0
        var intakeSupplyVoltage: Double = 0.0
        var intakeMotorVoltage: Double = 0.0
        var intakeStatorCurrent: Double = 0.0
        var exitSensor: Boolean = false
        var intakeSensor: Boolean = false

        /**
         * Updates a LogTable with the data to log.
         */
        override fun toLog(table: LogTable) {
            table.put("intakeLinearVelocity", intakeLinearVelocity)
            table.put("intakeSupplyVoltage", intakeSupplyVoltage)
            table.put("intakeMotorVoltage", intakeMotorVoltage)
            table.put("intakeStatorCurrent", intakeStatorCurrent)
            table.put("exitSensor", exitSensor)
            table.put("intakeSensor", intakeSensor)
        }

        /**
         * Updates data based on a LogTable.
         */
        override fun fromLog(table: LogTable) {
            intakeLinearVelocity = table.get("intakeLinearVelocity", intakeLinearVelocity)
            intakeSupplyVoltage = table.get("intakeSupplyVoltage", intakeSupplyVoltage)
            intakeMotorVoltage = table.get("intakeMotorVoltage", intakeMotorVoltage)
            intakeStatorCurrent = table.get("intakeStatorCurrent", intakeStatorCurrent)
            exitSensor = table.get("exitSensor", exitSensor)
            intakeSensor = table.get("intakeSensor", intakeSensor)
        }
    }
    fun updateInputs(inputs: IntakeInputs) {}
    fun applyVoltage(voltage: Measure<Voltage>) {}
    fun stop() {}
}
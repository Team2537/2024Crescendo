package frc.robot.subsystems.intake

import edu.wpi.first.units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO {
    class IntakeIOInputs : LoggableInputs {
        var intakeVelocity: MutableMeasure<Velocity<Angle>> =
            MutableMeasure.zero(Units.RadiansPerSecond)
        var intakeAppliedVoltage: MutableMeasure<Voltage> =
            MutableMeasure.zero(Units.Volts)
        var transferVelocity: MutableMeasure<Velocity<Angle>> =
            MutableMeasure.zero(Units.RadiansPerSecond)
        var transferAppliedVoltage: MutableMeasure<Voltage> =
            MutableMeasure.zero(Units.Volts)

        override fun toLog(table: LogTable?) {
            table?.put("intakeVelocity", intakeVelocity)
            table?.put("intakeAppliedVoltage", intakeAppliedVoltage)
            table?.put("transferVelocity", transferVelocity)
            table?.put("transferAppliedVoltage", transferAppliedVoltage)
        }

        override fun fromLog(table: LogTable?) {
            intakeVelocity.mut_replace(table?.get("intakeVelocity", intakeVelocity))
            intakeAppliedVoltage.mut_replace(table?.get("intakeAppliedVoltage", intakeAppliedVoltage))
            transferVelocity.mut_replace(table?.get("transferVelocity", transferVelocity))
            transferAppliedVoltage.mut_replace(table?.get("transferAppliedVoltage", transferAppliedVoltage))
        }

    }

    fun updateInputs(inputs: IntakeIOInputs)

    fun setIntakePower(power: Double)

    fun setTransferPower(power: Double)

    fun stopIntake()

    fun stopTransfer()
}
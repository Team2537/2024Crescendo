package frc.robot.subsystems.climb

import edu.wpi.first.units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ClimbIO {
    class ClimbIOInputs : LoggableInputs {
        var leftArmVelocity: MutableMeasure<Velocity<Distance>> =
            MutableMeasure.zero(Units.InchesPerSecond)
        var rightArmVelocity: MutableMeasure<Velocity<Distance>> =
            MutableMeasure.zero(Units.InchesPerSecond)
        var leftArmPosition: MutableMeasure<Distance> =
            MutableMeasure.zero(Units.Inches)
        var rightArmPosition: MutableMeasure<Distance> =
            MutableMeasure.zero(Units.Inches)
        var leftArmAppliedVoltage: MutableMeasure<Voltage> =
            MutableMeasure.zero(Units.Volts)
        var rightArmAppliedVoltage: MutableMeasure<Voltage> =
            MutableMeasure.zero(Units.Volts)


        override fun toLog(table: LogTable?) {
            table?.put("leftArmVelocity", leftArmVelocity)
            table?.put("rightArmVelocity", rightArmVelocity)
            table?.put("leftArmPosition", leftArmPosition)
            table?.put("rightArmPosition", rightArmPosition)
            table?.put("leftArmAppliedVoltage", leftArmAppliedVoltage)
            table?.put("rightArmAppliedVoltage", rightArmAppliedVoltage)
        }

        override fun fromLog(table: LogTable?) {
            leftArmVelocity.mut_replace(table?.get("leftArmVelocity", leftArmVelocity))
            rightArmVelocity.mut_replace(table?.get("rightArmVelocity", rightArmVelocity))
            leftArmPosition.mut_replace(table?.get("leftArmPosition", leftArmPosition))
            rightArmPosition.mut_replace(table?.get("rightArmPosition", rightArmPosition))
            leftArmAppliedVoltage.mut_replace(table?.get("leftArmAppliedVoltage", leftArmAppliedVoltage))
            rightArmAppliedVoltage.mut_replace(table?.get("rightArmAppliedVoltage", rightArmAppliedVoltage))
        }

    }

    fun updateInputs(inputs: ClimbIOInputs)

    fun setLeftArmPower(power: Double)

    fun setRightArmPower(power: Double)

    fun stop()

    fun resetLeftArmPosition()

    fun resetRightArmPosition()
}
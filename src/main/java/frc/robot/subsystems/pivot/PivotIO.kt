package frc.robot.subsystems.pivot

import edu.wpi.first.units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface PivotIO {
    class PivotIOInputs : LoggableInputs {
        var relativeAngle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Radians)
        var absoluteAngle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Radians)
        var homingSensorTriggered: Boolean = false
        var appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var setpoint: MutableMeasure<Angle> = MutableMeasure.zero(Units.Radians)
        override fun toLog(table: LogTable?) {
            table?.put("relativeAngle", relativeAngle)
            table?.put("absoluteAngle", absoluteAngle)
            table?.put("homingSensorTriggered", homingSensorTriggered)
            table?.put("appliedVoltage", appliedVoltage)
            table?.put("setpoint", setpoint)
        }

        override fun fromLog(table: LogTable?) {
            relativeAngle.mut_replace(table?.get("relativeAngle", relativeAngle))
            absoluteAngle.mut_replace(table?.get("absoluteAngle", absoluteAngle))
            homingSensorTriggered = table?.get("homingSensorTriggered", homingSensorTriggered) ?: false
            appliedVoltage.mut_replace(table?.get("appliedVoltage", appliedVoltage))
            setpoint.mut_replace(table?.get("setpoint", setpoint))
        }
    }

    fun updateInputs(inputs: PivotIOInputs)

    fun setRawVoltage(voltage: Measure<Voltage>)

    fun syncEncoders()

    fun zeroRelativeEncoder(position: Measure<Angle> = Units.Degrees.zero())

    fun runSetpoint(setpoint: Measure<Angle>, arbFFUnits: Measure<Voltage>)

    fun stop()

    fun getRawAbs(): Double
}
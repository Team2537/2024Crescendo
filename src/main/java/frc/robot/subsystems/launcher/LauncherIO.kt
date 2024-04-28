package frc.robot.subsystems.launcher

import com.revrobotics.SparkPIDController.ArbFFUnits
import edu.wpi.first.units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface LauncherIO {
    class LauncherIOInputs : LoggableInputs {
        var rollerVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
        var rollerPosition: MutableMeasure<Distance> = MutableMeasure.zero(Units.Meters)
        var topFlywheelsVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
        var bottomFlywheelsVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)
        var topFlywheelAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var bottomFlywheelAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var rightNoteDetected: Boolean = false

        override fun toLog(table: LogTable?) {
            table?.put("rollerVelocity", rollerVelocity)
            table?.put("flywheelsVelocity_t", topFlywheelsVelocity)
            table?.put("flywheelsVelocity_b", bottomFlywheelsVelocity)
            table?.put("noteDetected_r", rightNoteDetected)
            table?.put("rollerPosition", rollerPosition)
            table?.put("appliedVoltage_t", topFlywheelAppliedVoltage)
            table?.put("appliedVoltage_b", bottomFlywheelAppliedVoltage)
        }

        override fun fromLog(table: LogTable?) {
            rollerVelocity.mut_replace(table?.get("rollerVelocity", rollerVelocity))
            topFlywheelsVelocity.mut_replace(table?.get("flywheelsVelocity_t", topFlywheelsVelocity))
            bottomFlywheelsVelocity.mut_replace(table?.get("flywheelsVelocity_b", bottomFlywheelsVelocity))
            rightNoteDetected = table?.get("noteDetected_r", rightNoteDetected) ?: false
            rollerPosition.mut_replace(table?.get("rollerPosition", rollerPosition))
            topFlywheelAppliedVoltage.mut_replace(table?.get("appliedVoltage_t", topFlywheelAppliedVoltage))
            bottomFlywheelAppliedVoltage.mut_replace(table?.get("appliedVoltage_b", bottomFlywheelAppliedVoltage))
        }
    }

    fun updateInputs(inputs: LauncherIOInputs)

    fun setRollerPower(power: Double)

    fun setFlywheelPower(power: Double)

    fun runSetpoint(velocity: Measure<Velocity<Distance>>, arbFF_t: Double, arbFF_b: Double, arbFFUnits: ArbFFUnits)

    fun runRollerSetpoint(position: Measure<Distance>)

    fun setFlywheelsBrakeMode(brake: Boolean)

    fun setRollerBrakeMode(brake: Boolean)

    fun stopFlywheels()

    fun stopRoller()

    fun setFlywheelVoltage(voltage: Measure<Voltage>)

    fun setRollerVoltage(voltage: Measure<Voltage>)
}
package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import lib.math.units.into
import kotlin.math.PI

class IntakeIONeo(
    private val motorID: Int,
    private val intakeSensorID: Int,
    private val exitSensorID: Int,
    private val rollerDiameter: Measure<Distance>,
    private val gearing: Double,
) : IntakeIO {

    private val motor = CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = false
        encoder.positionConversionFactor = 1 / gearing
        encoder.velocityConversionFactor = 1 / gearing
        setSmartCurrentLimit(20) // Maximum safe limit on a neo 550
    }

    private val exitSensor: DigitalInput = DigitalInput(exitSensorID)
    private val intakeSensor: DigitalInput = DigitalInput(intakeSensorID)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.intakeSensorTriggered = intakeSensor.get()
        inputs.exitSensorTriggered = exitSensor.get()
        inputs.intakeLinearVelocity.mut_replace(
            motor.encoder.velocity * (((rollerDiameter into Meters) * PI) / 60.0), // Convert from RPM to m/s
            MetersPerSecond
        )
        inputs.intakeSupplyVoltage.mut_replace(motor.busVoltage, Volts)
        inputs.intakeMotorVoltage.mut_replace(motor.appliedOutput * motor.busVoltage, Volts)
        inputs.intakeStatorCurrent.mut_replace(motor.outputCurrent, Amps)
    }

    override fun setVoltage(voltage: Measure<Voltage>) {
        motor.setVoltage(voltage into Volts)
    }

    override fun stop() {
        motor.stopMotor()
    }

}
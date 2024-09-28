package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.DigitalInput
import lib.math.units.into
import kotlin.math.PI

class IntakeIONeo(
    private val motorID: Int,
    private val intakeSensorID: Int,
    private val exitSensorID: Int
) : IntakeIO {

    private val motor = CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(20)
    }

    private val exitSensor: DigitalInput = DigitalInput(exitSensorID)
    private val intakeSensor: DigitalInput = DigitalInput(intakeSensorID)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        inputs.intakeSensor = intakeSensor.get()
        inputs.exitSensor = exitSensor.get()
        inputs.intakeLinearVelocity = motor.encoder.velocity * ((Units.inchesToMeters(0.84) * PI)/ 60.0)
        inputs.intakeSupplyVoltage = motor.busVoltage
        inputs.intakeMotorVoltage = motor.appliedOutput * motor.busVoltage
        inputs.intakeStatorCurrent = motor.outputCurrent
    }

    override fun applyVoltage(voltage: Measure<Voltage>) {
        motor.setVoltage(voltage into Volts)
    }

    override fun stop() {
        motor.stopMotor()
    }

}
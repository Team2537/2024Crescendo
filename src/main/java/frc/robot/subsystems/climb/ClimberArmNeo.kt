package frc.robot.subsystems.climb

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import lib.ControllerGains
import lib.math.units.into
import lib.math.units.rotations
import lib.math.units.rpm

class ClimberArmNeo(
    private val id: Int,
    private val isInverted: Boolean,
    private val gearing: Double,
    private val drumRadius: Measure<Distance>,
): ClimberArmIO {
    private val motor: CANSparkMax = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless).apply {
        inverted = true
        encoder.positionConversionFactor = 1/gearing
        encoder.velocityConversionFactor = 1/gearing
        idleMode = CANSparkBase.IdleMode.kBrake
    }

    override fun updateInputs(inputs: ClimberArmIO.ClimberArmInputs) {
        inputs.velocity.mut_replace(
            (motor.encoder.velocity / 60) * (drumRadius into Meters) * 2 * Math.PI,
            MetersPerSecond
        )
        inputs.relativePosition.mut_replace(
            motor.encoder.position * (drumRadius into Meters) * 2 * Math.PI,
            Meters
        )
        inputs.appliedCurrent.mut_replace(Amps.of(motor.outputCurrent))
        inputs.appliedVoltage.mut_replace(Volts.of(motor.appliedOutput * motor.busVoltage))
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        motor.setVoltage(voltage into Volts)
    }

    override fun stop() {
        motor.stopMotor()
    }
}
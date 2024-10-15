package frc.robot.subsystems.launcher.roller

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import lib.ControllerGains
import lib.math.units.into

class RollerIONeo(
    private val motorID: Int,
    private val noteDetectorID: Int,
    private val isInverted: Boolean,
    private val gains: ControllerGains,
    private val rollerRadius: Measure<Distance>
) : RollerIO {
    private val motor: CANSparkMax = CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        inverted = isInverted

        setSmartCurrentLimit(40)
        pidController.setP(gains.kP)
        pidController.setI(gains.kI)
        pidController.setD(gains.kD)
    }

    private val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        gains.kS,
        gains.kV,
        gains.kA
    )

    private val noteDetector: DigitalInput = DigitalInput(noteDetectorID)

    override fun updateInputs(inputs: RollerIO.RollerInputs) {
        inputs.velocity.mut_replace(motor.encoder.velocity, RPM)
        inputs.position.mut_replace(motor.encoder.position, Rotations)
        inputs.statorCurrent.mut_replace(motor.outputCurrent, Amps)
        inputs.motorVoltage.mut_replace(motor.appliedOutput * motor.busVoltage, Volts)
        inputs.supplyVoltage.mut_replace(motor.busVoltage, Volts)
        inputs.noteDetected = noteDetector.get()
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        motor.setVoltage(voltage into Volts)
    }

    override fun setTargetPosition(position: Measure<Distance>) {
        val targetRotations = Units.radiansToRotations((position into Meters) / (rollerRadius into Meters))
        motor.pidController.setReference(targetRotations, CANSparkBase.ControlType.kPosition)
    }

    override fun setBrakeMode(isBrakeMode: Boolean) {
        motor.idleMode = if (isBrakeMode) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
    }
}
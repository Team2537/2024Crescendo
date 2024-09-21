package frc.robot.subsystems.pivot

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import lib.math.units.into

class PivotIONeos(
    private val pivotID: Int,
    private val pivotInverted: Boolean,
    private val absoluteEncoderID: Int,
    private val rotorToArmRatio: Double,
    private val encoderToArmRatio: Double,
    private val homingSensorID: Int,
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val kS: Double,
    private val kG: Double,
    private val kV: Double,
    private val kA: Double
) : PivotIO {
    private val pivotMotor: CANSparkMax = CANSparkMax(pivotID, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        inverted = pivotInverted
        encoder.positionConversionFactor = rotorToArmRatio
        encoder.velocityConversionFactor = rotorToArmRatio / 60.0

        pidController.setP(kP)
        pidController.setI(kI)
        pidController.setD(kD)

        setSmartCurrentLimit(40)
    }

    private val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(absoluteEncoderID).apply {
        distancePerRotation = encoderToArmRatio
    }

    private val feedforward: ArmFeedforward = ArmFeedforward(kS, kG, kV, kA)

    private val homingSensor: DigitalInput = DigitalInput(homingSensorID)

    override fun updateInputs(inputs: PivotIO.PivotInputs) {
        inputs.isAtHardstop = homingSensor.get()
        inputs.pivotRelativePosition.mut_replace(pivotMotor.encoder.position, Units.Rotations)
        inputs.pivotAbsolutePosition.mut_replace(absoluteEncoder.distance, Units.Rotations)
        inputs.pivotAngularVelocity.mut_replace(pivotMotor.encoder.velocity, Units.RotationsPerSecond)
        inputs.pivotMotorAppliedVoltage.mut_replace(pivotMotor.appliedOutput, Units.Volts)
        inputs.pivotMotorAppliedCurrent.mut_replace(pivotMotor.outputCurrent, Units.Amps)
    }

    override fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean) {
        pivotMotor.setVoltage(voltage into Units.Volts)
    }

    override fun setKnownPosition(position: Measure<Angle>) {
        pivotMotor.encoder.position = position into Units.Rotations
    }

    override fun setTargetPosition(position: Measure<Angle>) {
        pivotMotor.pidController.setReference(
            position into Units.Rotations,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(position into Units.Radians, 0.0)
        )
    }

    override fun setPID(p: Double, i: Double, d: Double) {
        pivotMotor.pidController.setP(p)
        pivotMotor.pidController.setI(i)
        pivotMotor.pidController.setD(d)
    }
}
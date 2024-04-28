package frc.robot.subsystems.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import frc.robot.Constants
import lib.math.units.rpm

class IntakeIONeos : IntakeIO {
    val intakeMotor: CANSparkBase = CANSparkMax(
        Constants.IntakeConstants.INTAKE_MOTOR_ID,
        CANSparkLowLevel.MotorType.kBrushless
    )

    val transferMotor: CANSparkBase = CANSparkMax(
        Constants.IntakeConstants.TRANSFER_MOTOR_ID,
        CANSparkLowLevel.MotorType.kBrushless
    )

    init {
        intakeMotor.restoreFactoryDefaults()
        transferMotor.restoreFactoryDefaults()

        intakeMotor.setSmartCurrentLimit(40)
        transferMotor.setSmartCurrentLimit(30)

        intakeMotor.burnFlash()
        transferMotor.burnFlash()
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.intakeVelocity = MutableMeasure.mutable(intakeMotor.encoder.velocity.rpm)
        inputs.transferVelocity = MutableMeasure.mutable(transferMotor.encoder.velocity.rpm)
        inputs.intakeAppliedVoltage = MutableMeasure.mutable(
            Units.Volts.of(intakeMotor.appliedOutput * intakeMotor.busVoltage)
        )
        inputs.transferAppliedVoltage = MutableMeasure.mutable(
            Units.Volts.of(transferMotor.appliedOutput * transferMotor.busVoltage)
        )
    }

    override fun setIntakePower(power: Double) {
        intakeMotor.set(power)
    }

    override fun setTransferPower(power: Double) {
        transferMotor.set(power)
    }

    override fun stopIntake() {
        intakeMotor.stopMotor()
    }

    override fun stopTransfer() {
        transferMotor.stopMotor()
    }
}
package frc.robot.subsystems.climb

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.Units
import frc.robot.Constants

class ClimbIONeos : ClimbIO {
    val leftArmMotor: CANSparkBase = CANSparkMax(
        Constants.ClimbConstants.LEFT_CLIMB_PORT,
        CANSparkLowLevel.MotorType.kBrushless
    )

    val rightArmMotor: CANSparkBase = CANSparkMax(
        Constants.ClimbConstants.RIGHT_CLIMB_PORT,
        CANSparkLowLevel.MotorType.kBrushless
    )

    init {
        leftArmMotor.restoreFactoryDefaults()
        rightArmMotor.restoreFactoryDefaults()

        leftArmMotor.setSmartCurrentLimit(40)
        rightArmMotor.setSmartCurrentLimit(40)

        leftArmMotor.encoder.positionConversionFactor = (1.0 / 16.0) * (Math.PI * 0.750)
        rightArmMotor.encoder.positionConversionFactor = (1.0 / 16.0) * (Math.PI * 0.750)

        leftArmMotor.encoder.velocityConversionFactor = (1.0 / 16.0) * (Math.PI * 0.750)
        rightArmMotor.encoder.velocityConversionFactor = (1.0 / 16.0) * (Math.PI * 0.750)

        leftArmMotor.encoder.position = 0.0
        rightArmMotor.encoder.position = 0.0

        leftArmMotor.idleMode = CANSparkBase.IdleMode.kBrake
        rightArmMotor.idleMode = CANSparkBase.IdleMode.kBrake

        leftArmMotor.burnFlash()
        rightArmMotor.burnFlash()
    }

    override fun updateInputs(inputs: ClimbIO.ClimbIOInputs) {
        inputs.leftArmPosition.mut_replace(leftArmMotor.encoder.position, Units.Inches)
        inputs.rightArmPosition.mut_replace(rightArmMotor.encoder.position, Units.Inches)
        inputs.leftArmVelocity.mut_replace(leftArmMotor.encoder.velocity, Units.InchesPerSecond)
        inputs.rightArmVelocity.mut_replace(rightArmMotor.encoder.velocity, Units.InchesPerSecond)
        inputs.leftArmAppliedVoltage.mut_replace(
            leftArmMotor.appliedOutput * leftArmMotor.busVoltage,
            Units.Volts
        )
        inputs.rightArmAppliedVoltage.mut_replace(
            rightArmMotor.appliedOutput * rightArmMotor.busVoltage,
            Units.Volts
        )
    }

    override fun setLeftArmPower(power: Double) {
        leftArmMotor.set(power)
    }

    override fun setRightArmPower(power: Double) {
        rightArmMotor.set(power)
    }

    override fun stop() {
        leftArmMotor.stopMotor()
        rightArmMotor.stopMotor()
    }

    override fun resetLeftArmPosition() {
        leftArmMotor.encoder.position = 0.0
    }

    override fun resetRightArmPosition() {
        rightArmMotor.encoder.position = 0.0
    }
}
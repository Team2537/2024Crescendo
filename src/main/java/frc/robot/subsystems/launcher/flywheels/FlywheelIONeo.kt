package frc.robot.subsystems.launcher.flywheels

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import lib.ControllerGains
import lib.math.units.into

class FlywheelIONeo(
    private val id: Int,
    private val isInverted: Boolean,
    private val gains: ControllerGains
) : FlywheelIO {
    private val motor = CANSparkFlex(id, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(40)
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = isInverted

        pidController.setP(gains.kP)
        pidController.setI(gains.kI)
        pidController.setD(gains.kD)

        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 10)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 10)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 32767)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, 32767)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 32767)
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, 32767)
    }

    private val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        gains.kS,
        gains.kV,
        gains.kA
    )

    override fun updateInputs(inputs: FlywheelIO.FlywheelInputs) {
        inputs.velocity.mut_replace(motor.encoder.velocity, RPM)
        inputs.position.mut_replace(motor.encoder.position, Rotations)
        inputs.statorCurrent.mut_replace(motor.outputCurrent, Amps)
        inputs.motorVoltage.mut_replace(motor.appliedOutput * motor.busVoltage, Volts)
        inputs.supplyVoltage.mut_replace(motor.busVoltage, Volts)
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        motor.setVoltage(voltage into Volts)
    }

    override fun setVelocity(velocity: Measure<Velocity<Angle>>) {
        motor.pidController.setReference(
            velocity into RPM,
            CANSparkBase.ControlType.kVelocity,
            0,
            feedforward.calculate(velocity into RPM),
            SparkPIDController.ArbFFUnits.kVoltage
        )
    }

    override fun setBrakeMode(isBrakeMode: Boolean) {
        motor.idleMode = if (isBrakeMode) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
    }
}
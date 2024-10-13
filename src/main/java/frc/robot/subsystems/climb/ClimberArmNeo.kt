package frc.robot.subsystems.climb

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.*

class ClimberArmNeo(private val id: Int, private val isInverted: Boolean): ClimberArmIO {
    private val motor: CANSparkMax = CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless).apply {
        inverted = true
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        TODO("Not yet implemented")
    }

    override fun setMotorVelocity(velocity: Measure<Velocity<Angle>>) {
        TODO("Not yet implemented")
    }

    override fun setLinearVelocity(velocity: Measure<Velocity<Distance>>) {
        TODO("Not yet implemented")
    }
}
package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface GyroIO {
    class GyroInputs : LoggableInputs {
        var connected: Boolean = false
        var yaw: Rotation2d = Rotation2d()
        var yawVelocityDegreesPerSecond: Double = 0.0
        var pitch: Rotation2d = Rotation2d()
        var pitchVelocityDegreesPerSecond: Double = 0.0
        var roll: Rotation2d = Rotation2d()
        var rollVelocityDegreesPerSecond: Double = 0.0
        override fun toLog(table: LogTable?) {
            table?.put("Connected", connected)
            table?.put("YawDegrees", Rotation2d.struct, yaw)
            table?.put("YawVelocityDegreesPerSecond", yawVelocityDegreesPerSecond)
            table?.put("PitchDegrees", Rotation2d.struct, pitch)
            table?.put("PitchVelocityDegreesPerSecond", pitchVelocityDegreesPerSecond)
            table?.put("RollDegrees", Rotation2d.struct, roll)
            table?.put("RollVelocityDegreesPerSecond", rollVelocityDegreesPerSecond)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("Connected")?.let { connected = it.boolean }
            table?.get("YawDegrees", Rotation2d.struct, Rotation2d())?.let { yaw = it }
            table?.get("YawVelocityDegreesPerSecond")?.let { yawVelocityDegreesPerSecond = it.double }
            table?.get("PitchDegrees", Rotation2d.struct, Rotation2d())?.let { pitch = it }
            table?.get("PitchVelocityDegreesPerSecond")?.let { pitchVelocityDegreesPerSecond = it.double }
            table?.get("RollDegrees", Rotation2d.struct, Rotation2d())?.let { roll = it }
            table?.get("RollVelocityDegreesPerSecond")?.let { rollVelocityDegreesPerSecond = it.double }
        }
    }

    fun updateInputs(inputs: GyroInputs) {}

    fun setYaw(newYaw: Double) {}
}

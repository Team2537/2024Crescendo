package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import java.util.function.Supplier

class GyroIOSim(private val speeds: Supplier<ChassisSpeeds>) : GyroIO {
    private var yawDegrees: Double = 0.0

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        val oldYaw = yawDegrees
        val robotSpeeds = speeds.get()
        inputs.connected = true
        inputs.yaw = inputs.yaw.plus(Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * 0.02))
        inputs.yawVelocityDegreesPerSecond = (yawDegrees - oldYaw) / 0.02
    }

    override fun setYaw(newYaw: Double) {
        yawDegrees = newYaw
    }
}

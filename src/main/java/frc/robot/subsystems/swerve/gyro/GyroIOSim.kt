package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import lib.math.units.degrees
import lib.math.units.degreesPerSecond
import java.util.function.Supplier

class GyroIOSim(private val speeds: Supplier<ChassisSpeeds>) : GyroIO {
    private var yaw: Rotation2d = Rotation2d()

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        val oldYaw = inputs.yaw
        val robotSpeeds = speeds.get()
        inputs.connected = true
        inputs.yaw = inputs.yaw.plus(Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * 0.02))
        inputs.yawVelocity.mut_replace((yaw.degrees - oldYaw.degrees) / 0.02, Units.DegreesPerSecond)
    }

    override fun setYaw(newYaw: Rotation2d) {
        yaw = newYaw
    }
}

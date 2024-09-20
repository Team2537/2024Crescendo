package frc.robot.subsystems.swerve.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants

class GyroIOPigeon2(private val canBusName: String = "", private val id: Int) : GyroIO {
    private val gyro: Pigeon2 = Pigeon2(id, canBusName)

    private val yawGetter: StatusSignal<Double> = gyro.yaw
    private val yawRateGetter: StatusSignal<Double> = gyro.angularVelocityZWorld
    private val pitchGetter: StatusSignal<Double> = gyro.pitch
    private val pitchRateGetter: StatusSignal<Double> = gyro.angularVelocityXWorld
    private val rollGetter: StatusSignal<Double> = gyro.roll
    private val rollRateGetter: StatusSignal<Double> = gyro.angularVelocityYWorld

    init {
        val gyroConfigs: Pigeon2Configuration = Pigeon2Configuration()
        gyro.configurator.setYaw(0.0)
        gyro.configurator.apply(gyroConfigs)
        yawGetter.setUpdateFrequency(100.0)
        yawRateGetter.setUpdateFrequency(100.0)
        pitchGetter.setUpdateFrequency(100.0)
        pitchRateGetter.setUpdateFrequency(100.0)
        rollGetter.setUpdateFrequency(100.0)
        rollRateGetter.setUpdateFrequency(100.0)
        gyro.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIO.GyroInputs) {
        inputs.connected =
            BaseStatusSignal.refreshAll(
                yawGetter,
                yawRateGetter,
            ).isOK

        inputs.yaw = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yawGetter, yawRateGetter))
        inputs.yawVelocityDegreesPerSecond = yawRateGetter.value
        inputs.pitch = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(pitchGetter, pitchRateGetter))
        inputs.pitchVelocityDegreesPerSecond = pitchRateGetter.value
        inputs.roll = Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(rollGetter, rollRateGetter))
        inputs.rollVelocityDegreesPerSecond = rollRateGetter.value
    }

    override fun setYaw(newYaw: Double) {
        gyro.setYaw(newYaw)
    }
}

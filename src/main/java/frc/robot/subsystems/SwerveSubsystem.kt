package frc.robot.subsystems


import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.swerve.SwerveModule
import swervelib.SwerveDrive

/**
 * The subsystem that controls the swerve drive.
 *
 * @constructor Creates a SwerveSubsystem from JSON Configuration Files.
 * @author Falon Clark
 * @since 1/15/2024
 */
object SwerveSubsystem : SubsystemBase() {
    var modules: Array<SwerveModule> = emptyArray()
    val pigeon: Pigeon2 = Pigeon2(13)

    val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(
        Constants.Drivebase.frontLeftModule.position,
        Constants.Drivebase.frontRightModule.position,
        Constants.Drivebase.backLeftModule.position,
        Constants.Drivebase.backRightModule.position
    )

    val odometer: SwerveDriveOdometry = SwerveDriveOdometry(kinematics, Rotation2d(0.0), getModulePositions())

    val poseEstimator: SwerveDrivePoseEstimator

    init {
        modules[0] = Constants.Drivebase.frontLeftModule
        modules[1] = Constants.Drivebase.frontRightModule
        modules[2] = Constants.Drivebase.backLeftModule
        modules[3] = Constants.Drivebase.backRightModule

        zeroModules()

        Thread(Runnable {
            Thread.sleep(500)
            resetIMU()
            zeroHeading()
        })
    }

    fun zeroHeading() {
        pigeon.setYaw(0.0)
    }

    fun getHeading(): Double {
        return pigeon.getYaw().valueAsDouble
    }

    fun resetIMU() {
        val configs: Pigeon2Configuration = Pigeon2Configuration()
        configs.MountPose.MountPoseYaw = 0.0
        configs.MountPose.MountPosePitch = 0.0
        configs.MountPose.MountPoseRoll = 0.0
        configs.Pigeon2Features.DisableNoMotionCalibration = false;
        configs.Pigeon2Features.DisableTemperatureCompensation = false;
        configs.Pigeon2Features.EnableCompass = false;

        pigeon.configurator.apply(configs)
    }

    fun zeroModules() {
        modules.forEach {
            it.zero()
        }
    }

    fun getPose() {
        // TODO: Implement Pose
    }

    fun getModulePositions(): Array<SwerveModulePosition> {
        val positions = arrayOf(
            SwerveModulePosition(
                Constants.Drivebase.frontLeftModule.driveEncoder.position,
                Constants.Drivebase.frontLeftModule.getState().angle
            ),
            SwerveModulePosition(
                Constants.Drivebase.frontRightModule.driveEncoder.position,
                Constants.Drivebase.frontRightModule.getState().angle
            ),
            SwerveModulePosition(
                Constants.Drivebase.backLeftModule.driveEncoder.position,
                Constants.Drivebase.backLeftModule.getState().angle
            ),
            SwerveModulePosition(
                Constants.Drivebase.backRightModule.driveEncoder.position,
                Constants.Drivebase.backRightModule.getState().angle
            )
        )

        return positions
    }
}

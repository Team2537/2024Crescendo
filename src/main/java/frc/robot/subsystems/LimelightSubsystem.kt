package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight
import lib.vision.LimelightHelpers
import lib.vision.VisionMeasurement

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {
    private val hostname: String = "limelight-odom"

    init {
        LimelightHelpers.setLEDMode_ForceOn(hostname)
    }

    fun getResults(): LimelightHelpers.LimelightResults {
        return LimelightHelpers.getLatestResults(hostname)
    }

    fun getTX(): Double {
        return LimelightHelpers.getTX(hostname)
    }

    fun getTY(): Double {
        return LimelightHelpers.getTY(hostname)
    }

    fun getTA(): Double {
        return LimelightHelpers.getTA(hostname)
    }

    fun getTV(): Boolean {
        return LimelightHelpers.getTV(hostname)
    }

    fun getPoseEsimate(): LimelightHelpers.PoseEstimate {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(hostname)
    }
}

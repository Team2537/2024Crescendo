package lib.vision

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import lib.math.units.Rotation
import lib.math.units.into
import lib.math.units.radians
import lib.vision.LimelightHelpers.LimelightResults
import lib.vision.LimelightHelpers.PoseEstimate
import kotlin.math.atan2
import kotlin.math.sqrt

class Limelight(private val hostname: String) {

    val results: LimelightResults
        get() = getLimelightResults()

    /**
     * Horizontal angle to the target
     */
    val tx: Double
        get() = getTX()

    /**
     * Vertical angle to the target
     */
    val ty: Double
        get() = getTY()

    /**
     * Percent of screen area taken up by the target
     */
    val ta: Double
        get() = getTA()

    /**
     * Distance from the limelight to the target
     */
    val td: Double
        get() = getTD()

    val distance2d: Double
        get() = getDist2D()

    val absoluteTX: Rotation
        get() = targetAngleAbsolute()

    /**
     * Whether a target is visible
     */
    val tv: Boolean
        get() = getTV()

    val poseEstimate: PoseEstimate
        get() = getBotPose()

    init {
        setLEDs(true)

        val tab = Shuffleboard.getTab("Vision")

        tab.addNumber("$hostname TX") { tx }
        tab.addNumber("$hostname TY") { ty }
        tab.addNumber("$hostname TA") { ta }
        tab.addNumber("$hostname TD") { td }
        tab.addNumber("$hostname TD2D") { distance2d }
        tab.addNumber("$hostname bot TX") { absoluteTX into Degrees }
        tab.addBoolean("$hostname TV") { tv }
    }

    private fun getLimelightResults(): LimelightResults {
        return LimelightHelpers.getLatestResults(hostname)
    }

    fun setLEDs(on: Boolean) {
        if (on) {
            LimelightHelpers.setLEDMode_ForceOn(hostname)
        } else {
            LimelightHelpers.setLEDMode_ForceOff(hostname)
        }
    }

    private fun getTX(): Double {
        return LimelightHelpers.getTX(hostname)
    }

    private fun getTY(): Double {
        return LimelightHelpers.getTY(hostname)
    }

    private fun getTA(): Double {
        return LimelightHelpers.getTA(hostname)
    }

    private fun getTV(): Boolean {
        return LimelightHelpers.getTV(hostname)
    }

    private fun targetAngleAbsolute(): Rotation {
        val targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(hostname)
        return atan2(targetPose.x, targetPose.z).radians
    }

    private fun getTD(): Double {
        val targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(hostname)

        // Distance of target from origin (should be robot location)
        return targetPose.translation.getDistance(Translation3d())
    }

    private fun getDist2D(): Double {
        val targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(hostname)

        return sqrt(targetPose.x * targetPose.x + targetPose.z * targetPose.z)
    }

    private fun getBotPose(): PoseEstimate {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(hostname)
    }

    /**
     * Prioritizes a specific april tag by the tag's ID
     */
    fun setTargetTag(id: Int) {
        LimelightHelpers.setPriorityTagID(hostname, id)
    }
}

package lib.vision

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import lib.math.units.degrees
import lib.math.units.into
import lib.math.units.pitch
import lib.vision.LimelightHelpers.LimelightResults
import lib.vision.LimelightHelpers.PoseEstimate

class Limelight(private val hostname: String) {

    val results: LimelightResults
        get() = getResults()

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

    /**
     * Whether a target is visible
     */
    val tv: Boolean
        get() = getTV()

    val poseEstimate: PoseEstimate
        get() = getPoseEstimate()

    init {
        LimelightHelpers.setLEDMode_ForceOn(hostname)

        val tab = Shuffleboard.getTab("Vision")

        tab.addNumber("$hostname TX") { tx }
        tab.addNumber("$hostname TY") { ty }
        tab.addNumber("$hostname TA") { ta }
        tab.addNumber("$hostname TD") { td }
        tab.addBoolean("$hostname TV") { tv }
    }

    private fun getResults(): LimelightResults {
        return LimelightHelpers.getLatestResults(hostname)
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

    private fun getTD(): Double {
        val targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(hostname)

        // Target distance from origin (should be robot location)
        return targetPose.translation.getDistance(Translation3d())
    }

    private fun getPoseEstimate(): PoseEstimate {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(hostname)
    }

    /**
     * Prioritizes a specific april tag by the tag's ID
     */
    fun setTargetTag(id: Int) {
        LimelightHelpers.setPriorityTagID(hostname, id)
    }
}

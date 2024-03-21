package lib.vision

import lib.vision.LimelightHelpers.LimelightResults
import lib.vision.LimelightHelpers.PoseEstimate

class Limelight(private val hostname: String) {

    val results: LimelightResults
        get() = getLLResults()

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
     * Whether a target is visible
     */
    val tv: Boolean
        get() = getTV()

    val poseEstimate: PoseEstimate
        get() = getLLPoseEstimate()

    init {
        LimelightHelpers.setLEDMode_ForceOn(hostname)
    }

    private fun getLLResults(): LimelightResults {
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

    private fun getLLPoseEstimate(): PoseEstimate {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(hostname)
    }

    /**
     * Prioritizes a specific april tag by the tag's ID
     */
    fun setTargetTag(id: Int) {
        LimelightHelpers.setPriorityTagID(hostname, id)
    }
}

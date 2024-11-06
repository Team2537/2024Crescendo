package lib.math.poseestimation

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.Kinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions
import edu.wpi.first.wpilibj.Timer
import java.util.NavigableMap
import java.util.TreeMap
import kotlin.math.pow

class TwistyPoseEstimator(val maxSampleAge: Double = 0.2, val sampleClusterTime: Double = 0.05) {


    private var preHistoryPose = Pose2d()
    private val history: NavigableMap<Double, List<WeightedTwist2d>> = TreeMap()
    
    var postHistoryPose = preHistoryPose
        private set


    var lastWheelMeasurement: SwerveDriveWheelPositions? = null

    private fun updateHistory(){
        if(history.isEmpty()){
            return
        }
        while(history.firstKey() < Timer.getFPGATimestamp() - maxSampleAge){
            history.pollFirstEntry().value.forEach {
                preHistoryPose = preHistoryPose.exp(it)
            }
        }
    }

    fun update(){
        updateHistory()
        postHistoryPose = preHistoryPose
        history.forEach { (_, twists) ->
            postHistoryPose = postHistoryPose.exp(fuseTwists(*twists.toTypedArray()))
        }
    }
    
    fun reset(pose: Pose2d){
        preHistoryPose = pose
        postHistoryPose = pose
        history.clear()
    }

    fun addWheelMeasurement(kinematics: SwerveDriveKinematics, reading: SwerveDriveWheelPositions, timestamp: Double, fom: Double){
        if(lastWheelMeasurement == null){
            lastWheelMeasurement = reading
            return
        }

        if(timestamp < Timer.getFPGATimestamp() - maxSampleAge){
            return
        }

        val twist = kinematics.toTwist2d(lastWheelMeasurement!!, reading)
        val weightedTwist = WeightedTwist2d(twist.dx, twist.dy, twist.dtheta, fom)

        // Find the closest timestamp within the sample cluster time
        val lastTimeStamp = history.floorKey(timestamp)
        if (lastTimeStamp != null && timestamp - lastTimeStamp <= sampleClusterTime) {
            // Append to the existing list at this timestamp
            history[lastTimeStamp] = history[lastTimeStamp]!! + weightedTwist
        } else {
            history[timestamp] = listOf(weightedTwist)
        }

        lastWheelMeasurement = reading
        updateHistory()
    }

    private fun fuseTwists(vararg twists: WeightedTwist2d): Twist2d {
        if(twists.isEmpty()) return Twist2d()

        val weightSum = twists.sumOf { 1.0 / it.fom.pow(2.0) }

        // Avoid division by zero in case all fom values are zero or extremely small
        if (weightSum == 0.0 || weightSum.isNaN() || weightSum.isInfinite() ) return Twist2d()

        return twists.fold(Twist2d()) { acc, twist ->
            val weight = 1.0 / twist.fom.pow(2.0)
            val normalizedWeight = weight / weightSum
            Twist2d(
                acc.dx + twist.dx * normalizedWeight,
                acc.dy + twist.dy * normalizedWeight,
                acc.dtheta + twist.dtheta * normalizedWeight
            )
        }
    }


}
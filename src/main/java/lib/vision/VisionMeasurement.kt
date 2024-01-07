package lib.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Timer

data class VisionMeasurement(val position: Pose3d, val timestamp: Double = Timer.getFPGATimestamp())

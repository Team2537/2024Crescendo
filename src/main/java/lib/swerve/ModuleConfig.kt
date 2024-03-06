package lib.swerve

import edu.wpi.first.math.geometry.Translation2d

data class ModuleConfig(
    val driveID: Int,
    val angleID: Int,
    val absoluteID: Int,
    val position: Translation2d,
    val offset: Double,
    val driveInverted: Boolean,
    val angleInverted: Boolean,
    val pid: ModulePID
)

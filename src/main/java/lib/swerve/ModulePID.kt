package lib.swerve

data class ModulePID(
    val driveKp: Double,
    val driveKi: Double,
    val driveKd: Double,
    val driveKf: Double,
    val driveIZone: Double,
    val angleKp: Double,
    val angleKi: Double,
    val angleKd: Double,
    val angleKf: Double,
    val angleIZone: Double
)

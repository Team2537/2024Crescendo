package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import frc.robot.Constants
import kotlin.math.cos

class SwerveModule(
    private val driveID: Int,
    private val turnID: Int,
    private val absoluteEncoderID: Int,
    private val invertDrive: Boolean,
    private val invertTurn: Boolean,
    private val invertAbsoluteEncoder: Boolean,
    private val absoluteOffset: Rotation2d,
) {
    private val io: ModuleIO
    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()

    val positionMeters: Double
        get() = inputs.drivePositionRads * Units.inchesToMeters(wheelRadiusInches)

    val velocityMetersPerSecond: Double
        get() = inputs.driveVelocityRadPerSec * Units.inchesToMeters(wheelRadiusInches)

    val positionRads: Double
        get() = inputs.drivePositionRads

    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    val state: SwerveModuleState
        get() = SwerveModuleState(velocityMetersPerSecond, angle)

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition(positionMeters, angle)

    init {
        io = when (Constants.RobotConstants.mode) {
            Constants.RobotConstants.Mode.REAL -> ModuleIONeo(
                driveID,
                turnID,
                absoluteEncoderID,
                invertDrive,
                invertTurn,
                invertAbsoluteEncoder,
                absoluteOffset,
                driveRatio,
                turnRatio
            )
            Constants.RobotConstants.Mode.SIM -> object : ModuleIO {}
            Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
        }

        io.reset()
    }

    fun apply(state: SwerveModuleState) {
        val optimized = SwerveModuleState.optimize(state, inputs.turnPosition)

//        println(optimized.speedMetersPerSecond)

        var speed = optimized.speedMetersPerSecond / Units.inchesToMeters(wheelRadiusInches)

        val steerError: Rotation2d = optimized.angle.minus(inputs.absoluteTurnPosition)
        speed *= cos(steerError.radians)

//        println("RadPerSec: $speed")

        io.runDriveVelocitySetpoint(speed)
        io.runTurnPositionSetpoint(optimized.angle.radians)
    }

    fun pointAt(angle: Rotation2d) {
        io.runTurnPositionSetpoint(angle.radians)
    }

    fun applyVoltage(voltage: Double) {
        io.runDriveVolts(voltage)
    }

    fun updateInputs() {
        io.updateInputs(inputs)
    }

    fun stop() {
        io.stop()
    }

    companion object {
        const val wheelRadiusInches: Double = 2.0
        const val driveRatio: Double = 6.75
        const val turnRatio: Double = 150.0/7.0
    }
}

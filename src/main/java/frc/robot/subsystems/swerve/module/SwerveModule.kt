package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import frc.robot.Constants
import lib.ControllerGains
import kotlin.math.cos

class SwerveModule(
    private val
    configs: ModuleIO.ModuleConstants,
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
                configs,
                ControllerGains(kV = 0.001684),
                ControllerGains(kP = 5.0, kD = 0.1)
            )

            Constants.RobotConstants.Mode.SIM -> ModuleIOSim(
                configs,
                DCMotor.getNEO(1),
                ControllerGains(),
                ControllerGains(kP = 25.0)
            )

            Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
        }

        io.reset()
    }

    fun apply(state: SwerveModuleState) {
        val optimized = SwerveModuleState.optimize(state, inputs.absoluteTurnPosition)

//        println(optimized.speedMetersPerSecond)

        var speed = optimized.speedMetersPerSecond / Units.inchesToMeters(wheelRadiusInches)

        val steerError: Rotation2d = optimized.angle.minus(inputs.absoluteTurnPosition)
        speed *= steerError.cos

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
    }
}

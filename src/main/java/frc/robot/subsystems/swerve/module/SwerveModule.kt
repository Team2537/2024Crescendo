package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.math.util.Units as Conversions
import frc.robot.Constants
import lib.ControllerGains
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

    val positionLinear: MutableMeasure<Distance>
        get() = positionLinear.mut_replace(inputs.drivePosition.times(wheelRadiusScalar) as Measure<Distance>)

    val velocityLinear: MutableMeasure<Velocity<Distance>>
        get() = velocityLinear.mut_replace(inputs.driveVelocity.times(wheelRadiusScalar) as Measure<Velocity<Distance>>)

    val position: MutableMeasure<Angle>
        get() = inputs.drivePosition

    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    val state: SwerveModuleState
        get() = SwerveModuleState(velocityLinear, angle)

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition(positionLinear, angle)

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
                turnRatio,
                ControllerGains(kP = 0.00023),
                ControllerGains(kP = 0.004)
            )
            Constants.RobotConstants.Mode.SIM -> object : ModuleIO {}
            Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
        }

        io.reset()
    }

    fun apply(state: SwerveModuleState) {
        val optimized = SwerveModuleState.optimize(state, inputs.turnPosition)

//        println(optimized.speedMetersPerSecond)

        var speed = optimized.speedMetersPerSecond / Conversions.inchesToMeters(wheelRadiusInches)

        val steerError: Rotation2d = optimized.angle.minus(inputs.absoluteTurnPosition)
        speed *= cos(steerError.radians)

//        println("RadPerSec: $speed")

        io.runDriveVelocitySetpoint(Units.RotationsPerSecond.of(speed))
        io.runTurnPositionSetpoint(optimized.angle)
    }

    fun pointAt(angle: Rotation2d) {
        io.runTurnPositionSetpoint(angle)
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
        val wheelRadiusScalar = Units.Inches.of(wheelRadiusInches).per(Units.Radians)
        const val driveRatio: Double = 6.75
        const val turnRatio: Double = 150.0/7.0
    }
}

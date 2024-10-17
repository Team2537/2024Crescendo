package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.Constants
import lib.ControllerGains

class SwerveModule(
    private val
    configs: ModuleIO.ModuleConstants,
) {

//    private val io: ModuleIO = when (Constants.RobotConstants.mode) {
//        Constants.RobotConstants.Mode.REAL -> ModuleIONeo(
//            configs,
//            ControllerGains(kV = 0.0113684210526),
//            ControllerGains(kP = 5.0, kD = 0.1)
//        )
//
//        Constants.RobotConstants.Mode.SIM -> ModuleIOSim(
//            configs,
//            DCMotor.getNEO(1),
//            ControllerGains(),
//            ControllerGains(kP = 10.0)
//        )
//
//        Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
//    }.apply { reset() }
    val io = object : ModuleIO {}


    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()

    val positionMeters: Double
        get() = inputs.drivePositionMeters

    val velocityMetersPerSecond: Double
        get() = inputs.driveVelocityMetersPerSec

    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    val state: SwerveModuleState
        get() = SwerveModuleState(velocityMetersPerSecond, angle)

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition(positionMeters, angle)

    fun apply(state: SwerveModuleState) {
        val optimized = SwerveModuleState.optimize(state, inputs.absoluteTurnPosition)

//        println(optimized.speedMetersPerSecond)

        var speed = optimized.speedMetersPerSecond

        val steerError: Rotation2d = optimized.angle.minus(inputs.absoluteTurnPosition)
        speed *= steerError.cos

//        println("RadPerSec: $speed")

        io.runDriveVelocitySetpoint(speed)
        io.runTurnPositionSetpoint(optimized.angle)
    }

    fun setDrivePID(p: Double, i: Double, d: Double) {
        io.setPID(p, i, d, ModuleIO.ModuleMotor.DRIVE)
    }

    fun setTurnPID(p: Double, i: Double, d: Double) {
        io.setPID(p, i, d, ModuleIO.ModuleMotor.TURN)
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
}

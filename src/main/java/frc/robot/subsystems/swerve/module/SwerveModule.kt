package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import frc.robot.Constants
import lib.ControllerGains
import lib.math.LoggedTunableNumber

class SwerveModule(
    private val
    configs: ModuleIO.ModuleConstants,
    private val ID: Int
) {
    private val io: ModuleIO
    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()

    val positionMeters: Double
        get() = inputs.drivePositionMeters

    val velocityMetersPerSecond: Double
        get() = inputs.drivePositionMetersPerSec

    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    val state: SwerveModuleState
        get() = SwerveModuleState(velocityMetersPerSecond, angle)

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition(positionMeters, angle)

    private val driveKp = LoggedTunableNumber("module[$ID]/drive/kP", 5.0)
    private val driveKi = LoggedTunableNumber("module[$ID]/drive/kI", 0.0)
    private val driveKd = LoggedTunableNumber("module[$ID]/drive/kD", 0.1)
    private val turnKp = LoggedTunableNumber("module[$ID]/turn/kP", 100.0)
    private val turnKi = LoggedTunableNumber("module[$ID]/turn/kI", 0.0)
    private val turnKd = LoggedTunableNumber("module[$ID]/turn/kD", 0.0)

    init {
        io = when (Constants.RobotConstants.mode) {
            Constants.RobotConstants.Mode.REAL -> ModuleIONeo(
                configs,
                ControllerGains(kV = 0.0113684210526),
                ControllerGains(kP = 5.0, kD = 0.1)
            )

            Constants.RobotConstants.Mode.SIM -> ModuleIOSim(
                configs,
                DCMotor.getNEO(1),
                ControllerGains(),
                ControllerGains(kP = 100.0)
            )

            Constants.RobotConstants.Mode.REPLAY -> object : ModuleIO {}
        }

        io.reset()
    }

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

    fun pointAt(angle: Rotation2d) {
        io.runTurnPositionSetpoint(angle)
    }

    fun applyVoltage(voltage: Double) {
        io.runDriveVolts(voltage)
    }

    fun updateInputs() {
        io.updateInputs(inputs)

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> io.setPID(pid[0], pid[1], pid[2], ModuleIO.ModuleMotor.DRIVE); println("Set PID: $pid") },
            driveKp, driveKi, driveKd
        )
        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> io.setPID(pid[0], pid[1], pid[2], ModuleIO.ModuleMotor.TURN); println("Set PID: $pid") },
            turnKp, turnKi, turnKd
        )
    }

    fun stop() {
        io.stop()
    }
}

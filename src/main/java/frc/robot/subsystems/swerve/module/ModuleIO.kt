package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Swerve module IO interface
 *
 * This interface defines the methods that a swerve module must implement in order to be used in the swerve drive
 */
interface ModuleIO {

    enum class ModuleMotor {
        DRIVE, TURN
    }

    /**
     * The inputs for the module
     *
     * This class is used to store the current state of the module's sensors and motors
     */
    class ModuleInputs : LoggableInputs {
        var driveMotorConnected: Boolean = false
        var turnMotorConnected: Boolean = false

        var drivePositionRads: Double = 0.0
        var driveVelocityRadPerSec: Double = 0.0
        var driveSupplyVolts: Double = 0.0
        var driveMotorVolts: Double = 0.0
        var driveStatorCurrent: Double = 0.0
        var driveSupplyCurrent: Double = 0.0

        var turnPosition: Rotation2d = Rotation2d()
        var absoluteTurnPosition: Rotation2d = Rotation2d()
        var turnVelocityRadPerSec: Double = 0.0
        var turnSupplyVolts: Double = 0.0
        var turnMotorVolts: Double = 0.0
        var turnStatorCurrent: Double = 0.0
        var turnSupplyCurrent: Double = 0.0

        /** @suppress */
        override fun toLog(table: LogTable?) {
            table?.put("driveMotorConnected", driveMotorConnected)
            table?.put("turnMotorConnected", turnMotorConnected)

            table?.put("drivePositionRads", drivePositionRads)
            table?.put("driveVelocityRadPerSec", driveVelocityRadPerSec)
            table?.put("driveSupplyVolts", driveSupplyVolts)
            table?.put("driveMotorVolts", driveMotorVolts)
            table?.put("driveStatorCurrent", driveStatorCurrent)
            table?.put("driveSupplyCurrent", driveSupplyCurrent)

            table?.put("turnPosition", Rotation2d.struct, turnPosition)
            table?.put("absoluteTurnPosition", Rotation2d.struct, absoluteTurnPosition)
            table?.put("turnVelocityRadPerSec", turnVelocityRadPerSec)
            table?.put("turnSupplyVolts", turnSupplyVolts)
            table?.put("turnMotorVolts", turnMotorVolts)
            table?.put("turnStatorCurrent", turnStatorCurrent)
            table?.put("turnSupplyCurrent", turnSupplyCurrent)
        }

        /** @suppress */
        override fun fromLog(table: LogTable?) {
            table?.get("driveMotorConnected")?.let { driveMotorConnected = it.boolean }
            table?.get("turnMotorConnected")?.let { turnMotorConnected = it.boolean }

            table?.get("drivePositionRads")?.let { drivePositionRads = it.double }
            table?.get("driveVelocityRadPerSec")?.let { driveVelocityRadPerSec = it.double }
            table?.get("driveSupplyVolts")?.let { driveSupplyVolts = it.double }
            table?.get("driveMotorVolts")?.let { driveMotorVolts = it.double }
            table?.get("driveStatorCurrent")?.let { driveStatorCurrent = it.double }
            table?.get("driveSupplyCurrent")?.let { driveSupplyCurrent = it.double }

            table?.get("turnPosition", Rotation2d.struct, Rotation2d())?.let { turnPosition = it }
            table?.get("absoluteTurnPosition", Rotation2d.struct, Rotation2d())?.let { absoluteTurnPosition = it }
            table?.get("turnVelocityRadPerSec")?.let { turnVelocityRadPerSec = it.double }
            table?.get("turnSupplyVolts")?.let { turnSupplyVolts = it.double }
            table?.get("turnMotorVolts")?.let { turnMotorVolts = it.double }
            table?.get("turnStatorCurrent")?.let { turnStatorCurrent = it.double }
            table?.get("turnSupplyCurrent")?.let { turnSupplyCurrent = it.double }
        }
    }

    data class ModuleConstants(
        val driveID: Int,
        val turnID: Int,
        val absoluteEncoderID: Int,
        val invertDrive: Boolean,
        val invertTurn: Boolean,
        val invertAbsoluteEncoder: Boolean,
        val absoluteOffset: Rotation2d,
        val turnRatio: Double,
        val driveRatio: Double,
    )

    /**
     * Update the inputs using the current state of the module
     * @param inputs The inputs to update (mutated in place)
     */
    fun updateInputs(inputs: ModuleInputs) {}

    /**
     * Run the drive motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    fun runDriveVolts(volts: Double) {}

    /**
     * Run the turn motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    fun runTurnVolts(volts: Double) {}

    /**
     * Set the position setpoint for the turn motor
     *
     * @param positionRads The position to set the motor to in radians
     */
    fun runTurnPositionSetpoint(positionRads: Double) {}

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityRadPerSec The velocity to set the motor to in radians per second
     */
    fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {}

    /**
     * Stop the module
     */
    fun stop() {}

    /**
     * Reset the module's position, for odometry purposes
     */
    fun reset() {}

    /**
     * Set the PID constants for a motor
     *
     * @param p The proportional constant
     * @param i The integral constant
     * @param d The derivative constant
     * @param motor The motor to set the constants for
     */
    fun setPID(p: Double, i: Double, d: Double, motor: ModuleMotor) {}

    /**
     * Set the feedforward constants for a motor
     *
     * @param kV The velocity feedforward constant
     * @param kA The acceleration feedforward constant
     * @param kS The static feedforward constant
     * @param motor The motor to set the constants for
     */
    fun setFF(kV: Double, kA: Double, kS: Double, motor: ModuleMotor) {}
}

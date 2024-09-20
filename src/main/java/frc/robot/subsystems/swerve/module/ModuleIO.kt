package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
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

        var drivePosition: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var driveVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var driveSupplyVolts: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var driveMotorVolts: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var driveStatorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)
        var driveSupplyCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)

        var turnPosition: Rotation2d = Rotation2d()
        var absoluteTurnPosition: Rotation2d = Rotation2d()
        var turnVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var turnSupplyVolts: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var turnMotorVolts: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var turnStatorCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)
        var turnSupplyCurrent: MutableMeasure<Current> = MutableMeasure.zero(Units.Amps)

        /** @suppress */
        override fun toLog(table: LogTable?) {
            table?.put("driveMotorConnected", driveMotorConnected)
            table?.put("turnMotorConnected", turnMotorConnected)

            table?.put("drivePosition", drivePosition)
            table?.put("driveVelocity", driveVelocity)
            table?.put("driveSupplyVolts", driveSupplyVolts)
            table?.put("driveMotorVolts", driveMotorVolts)
            table?.put("driveStatorCurrent", driveStatorCurrent)
            table?.put("driveSupplyCurrent", driveSupplyCurrent)

            table?.put("turnPosition", Rotation2d.struct, turnPosition)
            table?.put("absoluteTurnPosition", Rotation2d.struct, absoluteTurnPosition)
            table?.put("turnVelocity", turnVelocity)
            table?.put("turnSupplyVolts", turnSupplyVolts)
            table?.put("turnMotorVolts", turnMotorVolts)
            table?.put("turnStatorCurrent", turnStatorCurrent)
            table?.put("turnSupplyCurrent", turnSupplyCurrent)
        }

        /** @suppress */
        override fun fromLog(table: LogTable) {
            table?.get("driveMotorConnected")?.let { driveMotorConnected = it.boolean }
            table?.get("turnMotorConnected")?.let { turnMotorConnected = it.boolean }

            drivePosition.mut_replace(table.get("drivePosition", drivePosition))
            driveVelocity.mut_replace(table.get("driveVelocity", driveVelocity))
            driveSupplyVolts.mut_replace(table.get("driveSupplyVolts", driveSupplyVolts))
            driveMotorVolts.mut_replace(table.get("driveMotorVolts", driveMotorVolts))
            driveStatorCurrent.mut_replace(table.get("driveStatorCurrent", driveStatorCurrent))
            driveSupplyCurrent.mut_replace(table.get("driveSupplyCurrent", driveSupplyCurrent))

            table?.get("turnPosition", Rotation2d.struct, Rotation2d())?.let { turnPosition = it }
            table?.get("absoluteTurnPosition", Rotation2d.struct, Rotation2d())?.let { absoluteTurnPosition = it }
            turnVelocity.mut_replace(table.get("turnVelocity", turnVelocity))
            turnSupplyVolts.mut_replace(table.get("turnSupplyVolts", turnSupplyVolts))
            turnMotorVolts.mut_replace(table.get("turnMotorVolts", turnMotorVolts))
            turnStatorCurrent.mut_replace(table.get("turnStatorCurrent", turnStatorCurrent))
            turnSupplyCurrent.mut_replace(table.get("turnSupplyCurrent", turnSupplyCurrent))
        }
    }

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
     * @param position The position to set the motor to
     */
    fun runTurnPositionSetpoint(position: Rotation2d) {}

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityRadPerSec The velocity to set the motor to in radians per second
     */
    fun runDriveVelocitySetpoint(velocity: Measure<Velocity<Angle>>) {}

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

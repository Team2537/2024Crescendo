package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import lib.ControllerGains
import lib.math.units.into

class ModuleIONeo(
    private val driveID: Int,
    private val turnID: Int,
    private val absoluteEncoderID: Int,
    private val invertDrive: Boolean,
    private val invertTurn: Boolean,
    private val invertAbsoluteEncoder: Boolean,
    private val absoluteOffset: Rotation2d,
    private val driveRatio: Double,
    private val turnRatio: Double,
    private val driveGains: ControllerGains,
    private val turnGains: ControllerGains,
) : ModuleIO {

    private val driveMotor = CANSparkMax(driveID, CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = invertDrive
        encoder.positionConversionFactor = driveRatio / 60.0
        encoder.velocityConversionFactor = driveRatio / 60.0

        pidController.p = driveGains.kP
        pidController.i = driveGains.kI
        pidController.d = driveGains.kD
        pidController.ff = driveGains.kV
    }

    private val turnMotor = CANSparkMax(turnID, CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = invertTurn
        encoder.positionConversionFactor = turnRatio / 60.0
        encoder.velocityConversionFactor = turnRatio / 60.0

        pidController.p = turnGains.kP
        pidController.i = turnGains.kI
        pidController.d = turnGains.kD
    }

    private val absoluteEncoder: CANcoder = CANcoder(absoluteEncoderID).apply {
        val config = CANcoderConfiguration()
        config.MagnetSensor.MagnetOffset = absoluteOffset.rotations
        configurator.apply(config)
    }
    private val encoderPositionSignal: StatusSignal<Double> = absoluteEncoder.position.clone()

    private var driveKs = driveGains.kS
    private var turnKs = turnGains.kS

    /**
     * Update the inputs using the current state of the module
     * @param inputs The inputs to update (mutated in place)
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true

        inputs.drivePosition.mut_replace(driveMotor.encoder.position, Units.Rotations)
        inputs.driveVelocity.mut_replace(driveMotor.encoder.velocity, Units.RotationsPerSecond)
        inputs.driveSupplyVolts.mut_replace(driveMotor.busVoltage, Units.Volts)
        inputs.driveMotorVolts.mut_replace(driveMotor.appliedOutput * driveMotor.busVoltage, Units.Volts)
        inputs.driveStatorCurrent.mut_replace(driveMotor.outputCurrent, Units.Amps)
        inputs.driveSupplyCurrent.mut_replace(driveMotor.outputCurrent, Units.Amps)

        inputs.turnPosition = Rotation2d.fromRotations(turnMotor.encoder.position)
        inputs.absoluteTurnPosition = Rotation2d.fromRotations(encoderPositionSignal.value)
        inputs.turnVelocity.mut_replace(turnMotor.encoder.velocity, Units.RotationsPerSecond)
        inputs.turnSupplyVolts.mut_replace(turnMotor.busVoltage, Units.Volts)
        inputs.turnMotorVolts.mut_replace(turnMotor.appliedOutput * turnMotor.busVoltage, Units.Volts)
        inputs.turnStatorCurrent.mut_replace(turnMotor.outputCurrent, Units.Amps)
        inputs.turnSupplyCurrent.mut_replace(turnMotor.outputCurrent, Units.Amps)
    }

    /**
     * Run the drive motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runDriveVolts(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    /**
     * Run the turn motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runTurnVolts(volts: Double) {
        turnMotor.setVoltage(volts)
    }

    /**
     * Set the position setpoint for the turn motor
     *
     * @param positionRads The position to set the motor to in radians
     */
    override fun runTurnPositionSetpoint(position: Rotation2d) {
        turnMotor.pidController.setReference(
            position.rotations,
            CANSparkBase.ControlType.kPosition,
            0,
            turnKs,
        )
    }

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityRadPerSec The velocity to set the motor to in radians per second
     */
    override fun runDriveVelocitySetpoint(velocity: Measure<Velocity<Angle>>) {
        driveMotor.pidController.setReference(
            velocity into Units.RotationsPerSecond,
            CANSparkBase.ControlType.kVelocity,
            0,
            driveKs,
        )
    }

    /**
     * Stop the module
     */
    override fun stop() {
        driveMotor.stopMotor()
        turnMotor.stopMotor()
    }

    /**
     * Reset the module's position, for odometry purposes
     */
    override fun reset() {
        driveMotor.encoder.position = 0.0
    }

    /**
     * Set the PID constants for a motor
     *
     * @param p The proportional constant
     * @param i The integral constant
     * @param d The derivative constant
     * @param motor The motor to set the constants for
     */
    override fun setPID(p: Double, i: Double, d: Double, motor: ModuleIO.ModuleMotor) {
        when (motor) {
            ModuleIO.ModuleMotor.DRIVE -> {
                driveMotor.pidController.p = p
                driveMotor.pidController.i = i
                driveMotor.pidController.d = d
            }
            ModuleIO.ModuleMotor.TURN -> {
                turnMotor.pidController.p = p
                turnMotor.pidController.i = i
                turnMotor.pidController.d = d
            }
        }
    }

    /**
     * Set the feedforward constants for a motor
     *
     * @param kV The velocity feedforward constant
     * @param kA The acceleration feedforward constant
     * @param kS The static feedforward constant
     * @param motor The motor to set the constants for
     */
    override fun setFF(kV: Double, kA: Double, kS: Double, motor: ModuleIO.ModuleMotor) {
        when (motor) {
            ModuleIO.ModuleMotor.DRIVE -> {
                driveMotor.pidController.ff = kV
                driveKs = kS
            }
            ModuleIO.ModuleMotor.TURN -> {
                turnMotor.pidController.ff = kV
                turnKs = kS
            }
        }
    }
}

package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import lib.ControllerGains

class ModuleIONeo(
    private val configs: ModuleIO.ModuleConstants,
    private val driveGains: ControllerGains,
    private val turnGains: ControllerGains,
) : ModuleIO {

    private val driveMotor = CANSparkMax(configs.driveID, CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = configs.invertDrive
        encoder.positionConversionFactor = 1 / configs.driveRatio
        encoder.velocityConversionFactor = 1 / configs.driveRatio

        pidController.p = driveGains.kP
        pidController.i = driveGains.kI
        pidController.d = driveGains.kD
        pidController.ff = driveGains.kV

        setSmartCurrentLimit(40)
        enableVoltageCompensation(12.0)
    }

    private val turnMotor = CANSparkMax(configs.turnID, CANSparkLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkBase.IdleMode.kBrake
        inverted = configs.invertTurn
        encoder.positionConversionFactor = 1 / configs.turnRatio
        encoder.velocityConversionFactor = 1 / configs.turnRatio

        pidController.p = turnGains.kP
        pidController.i = turnGains.kI
        pidController.d = turnGains.kD

        pidController.setPositionPIDWrappingEnabled(true)
        pidController.positionPIDWrappingMaxInput = 0.5
        pidController.positionPIDWrappingMinInput = -0.5

        setSmartCurrentLimit(30)
        enableVoltageCompensation(12.0)
    }

    private val absoluteEncoder: CANcoder = CANcoder(configs.absoluteEncoderID).apply {
        val config = CANcoderConfiguration()
        config.MagnetSensor.MagnetOffset = configs.absoluteOffset.rotations
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

        inputs.drivePositionRads = Units.rotationsToRadians(driveMotor.encoder.position)
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveMotor.encoder.velocity)
        inputs.driveSupplyVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveMotorVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveStatorCurrent = driveMotor.outputCurrent
        inputs.driveSupplyCurrent = driveMotor.outputCurrent

        inputs.turnPosition = Rotation2d.fromRotations(turnMotor.encoder.position)
        inputs.absoluteTurnPosition = Rotation2d.fromRotations(encoderPositionSignal.value)
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnMotor.encoder.velocity)
        inputs.turnSupplyVolts = turnMotor.appliedOutput * turnMotor.busVoltage
        inputs.turnMotorVolts = turnMotor.appliedOutput * turnMotor.busVoltage
        inputs.turnStatorCurrent = turnMotor.outputCurrent
        inputs.turnSupplyCurrent = turnMotor.outputCurrent
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
    override fun runTurnPositionSetpoint(positionRads: Double) {
        val positionRotations = Units.radiansToRotations(positionRads)
        turnMotor.pidController.setReference(
            positionRotations,
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
    override fun runDriveVelocitySetpoint(velocityRadPerSec: Double) {
        val velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec)
        driveMotor.pidController.setReference(
            velocityRPM,
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

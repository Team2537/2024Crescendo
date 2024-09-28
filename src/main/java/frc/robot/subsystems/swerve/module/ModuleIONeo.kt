package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.SimpleMotorFeedforward
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

        setSmartCurrentLimit(40)
        enableVoltageCompensation(12.0)
    }


    private val absoluteEncoder: CANcoder = CANcoder(configs.absoluteEncoderID).apply {
        val config = CANcoderConfiguration()
        config.MagnetSensor.MagnetOffset = configs.absoluteOffset.rotations
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf
        configurator.apply(config)
    }

    private val encoderPositionSignal: StatusSignal<Double> = absoluteEncoder.absolutePosition.clone()

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

        encoder.setPosition(encoderPositionSignal.valueAsDouble)

        setSmartCurrentLimit(30)
        enableVoltageCompensation(12.0)
    }


    private val driveFF: SimpleMotorFeedforward = SimpleMotorFeedforward(driveGains.kS, driveGains.kV)
    private var turnKs = turnGains.kS

    /**
     * Update the inputs using the current state of the module
     * @param inputs The inputs to update (mutated in place)
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true
        inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(encoderPositionSignal).isOK

        inputs.drivePositionMeters =
            Units.rotationsToRadians(driveMotor.encoder.position) * configs.wheelRadiusInches
        inputs.drivePositionMetersPerSec =
            Units.rotationsPerMinuteToRadiansPerSecond(driveMotor.encoder.velocity) * configs.wheelRadiusInches
        inputs.driveSupplyVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveMotorVolts = driveMotor.appliedOutput * driveMotor.busVoltage
        inputs.driveStatorCurrent = driveMotor.outputCurrent
        inputs.driveSupplyCurrent = driveMotor.outputCurrent

        inputs.absoluteTurnPosition = Rotation2d.fromRotations(encoderPositionSignal.valueAsDouble)
        inputs.turnPosition = Rotation2d.fromRotations(turnMotor.encoder.position)
        inputs.turnVelocityRotationsPerSec = turnMotor.encoder.velocity / 60.0
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
     * @param position The position to set the motor to in radians
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
     * @param velocityMetersPerSecond The velocity to set the motor to in radians per second
     */
    override fun runDriveVelocitySetpoint(velocityMetersPerSecond: Double) {
        // Convert m/s to RPM
        val velocityRPM = (velocityMetersPerSecond * 60) / (2 * Math.PI * configs.wheelRadiusInches)

        // Set the reference for the motor's PID controller
        driveMotor.pidController.setReference(
            velocityRPM,
            CANSparkBase.ControlType.kVelocity,
            0,
            driveFF.calculate(velocityRPM)
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
}

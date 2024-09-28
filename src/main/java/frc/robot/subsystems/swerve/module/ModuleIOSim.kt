package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.ControllerGains

class ModuleIOSim(
    private val configs: ModuleIO.ModuleConstants,
    private val motor: DCMotor,
    private val driveGains: ControllerGains,
    private val turnGains: ControllerGains,
) : ModuleIO {
    /**
     * Motor sim for the drive motor
     *
     * jkgMetersSquares is pulled out of thin air, but it works
     */
    private val driveMotorSim: DCMotorSim =
        DCMotorSim(motor, configs.driveRatio, 0.025)

    /**
     * Motor sim for the turn motor
     *
     * jkgMetersSquares is pulled out of thin air, but it works
     */
    private val turnMotorSim: DCMotorSim =
        DCMotorSim(motor, configs.turnRatio, 0.004)

    /** PID controller for drive motor velocity control */
    private val driveFeedback = PIDController(driveGains.kP, driveGains.kI, driveGains.kD)

    /** PID controller for turn motor position control (crank that shiz) */
    private val turnFeedback = PIDController(turnGains.kP, turnGains.kI, turnGains.kD)

    /** Auto-generated kV for sim */
    private val driveKv = 12.0 / 4.0

    // kV is just the slope of a linear regression on the Points 0,0 and 77.17 (Max speed in radians), 12 (Max voltage)
    /**
     * Feedforward object for drive motor velocity control
     */
    private val driveFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, driveKv)

    /**
     * Feedforward object for turn motor position control
     */
    private val turnFeedforward: SimpleMotorFeedforward =
        SimpleMotorFeedforward(turnGains.kS, turnGains.kV, turnGains.kA)

    private var driveAppliedVolts: Double = 0.0
    private var steerAppliedVolts: Double = 0.0
    private val turnAbsoluteInitPosition: Rotation2d = configs.absoluteOffset

    init {
        turnFeedback.enableContinuousInput(-0.5, 0.5)
        println(driveKv)
    }

    /**
     * Update the inputs using the current state of the module
     * @param inputs The inputs to update (mutated in place)
     */
    override fun updateInputs(inputs: ModuleIO.ModuleInputs) {
        driveMotorSim.update(0.02)
        turnMotorSim.update(0.02)

        inputs.driveMotorConnected = true
        inputs.turnMotorConnected = true
        inputs.absoluteEncoderConnected = true

        inputs.drivePositionMeters = Units.inchesToMeters(configs.wheelRadiusInches) * driveMotorSim.angularPositionRad
        inputs.drivePositionMetersPerSec =
            Units.inchesToMeters(configs.wheelRadiusInches) * driveMotorSim.angularVelocityRadPerSec
        inputs.driveSupplyVolts = driveAppliedVolts
        inputs.driveMotorVolts = driveAppliedVolts
        inputs.driveStatorCurrent = driveMotorSim.currentDrawAmps
        inputs.driveSupplyCurrent = driveMotorSim.currentDrawAmps

        inputs.turnPosition = Rotation2d.fromRadians(turnMotorSim.angularPositionRad)
        inputs.absoluteTurnPosition = Rotation2d.fromRadians(turnMotorSim.angularPositionRad)
        inputs.turnVelocityRotationsPerSec = Units.radiansToRotations(turnMotorSim.angularVelocityRadPerSec)
        inputs.turnSupplyVolts = steerAppliedVolts
        inputs.turnMotorVolts = steerAppliedVolts
        inputs.turnStatorCurrent = turnMotorSim.currentDrawAmps
        inputs.turnSupplyCurrent = turnMotorSim.currentDrawAmps
    }

    /**
     * Run the drive motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runDriveVolts(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        driveMotorSim.setInputVoltage(driveAppliedVolts)
    }

    /**
     * Run the turn motor at a given voltage
     *
     * @param volts The voltage to run the motor at
     */
    override fun runTurnVolts(volts: Double) {
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        turnMotorSim.setInputVoltage(steerAppliedVolts)
    }

    /**
     * Set the position setpoint for the turn motor
     *
     * @param position The position to set the motor to in radians
     */
    override fun runTurnPositionSetpoint(position: Rotation2d) {
        runTurnVolts(
            turnFeedback.calculate(turnMotorSim.angularPositionRad, position.rotations) + turnFeedforward.calculate(
                position.rotations
            ),
        )
    }

    /**
     * Set the velocity setpoint for the drive motor
     *
     * @param velocityMetersPerSecond The velocity to set the motor to in radians per second
     */
    override fun runDriveVelocitySetpoint(velocityMetersPerSecond: Double) {
        runDriveVolts(
            driveFeedback.calculate(
                driveMotorSim.angularVelocityRadPerSec,
                velocityMetersPerSecond
            ) + driveFeedforward.calculate(velocityMetersPerSecond),
        )
    }

    /**
     * Stop the module
     */
    override fun stop() {
        runTurnVolts(0.0)
        runDriveVolts(0.0)
    }

    /**
     * Reset the module's position, for odometry purposes
     */
    override fun reset() {
        driveMotorSim.setState(0.0, 0.0)
    }
}
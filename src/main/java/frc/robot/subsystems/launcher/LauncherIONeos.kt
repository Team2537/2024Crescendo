package frc.robot.subsystems.launcher

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.Constants
import lib.math.units.into
import lib.math.units.radiansPerSecond

class LauncherIONeos(
        topFlywheelsId: Int,
        bottomFlywheelsId: Int,
        rollerId: Int,
        noteDetectorID: Int,
        /**
         * The radius of the flywheels, used for converting rotational to linear velocity
         */
        private val flywheelRadius: Measure<Distance>,
) : LauncherIO {

    private val topFlywheels: CANSparkFlex = CANSparkFlex(topFlywheelsId, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(40)

        idleMode = CANSparkBase.IdleMode.kBrake
    }

    // Grab the encoder once because getEncoder() has some object lock overhead
    private val topFlywheelsEncoder = topFlywheels.encoder

    // Similar story here
    private val topFlywheelsPIDController = topFlywheels.pidController

    private var topFlywheelsFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private val bottomFlywheels: CANSparkFlex = CANSparkFlex(bottomFlywheelsId, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(40)

        inverted = true

        // maybe?
        //follow(topFlywheels)

        idleMode = CANSparkBase.IdleMode.kBrake
    }

    private val bottomFlywheelsEncoder = bottomFlywheels.encoder

    private val bottomFlywheelsPIDController = bottomFlywheels.pidController

    private var bottomFlywheelsFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private val roller: CANSparkFlex = CANSparkFlex(rollerId, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        setSmartCurrentLimit(40)
    }

    private val rollerEncoder = roller.encoder

    private val rollerPIDController = roller.pidController

    private var rollerFeedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    private val noteDetector = DigitalInput(noteDetectorID)

    /**
     * Updates the given inputs with data from this IO layer
     */
    override fun updateInputs(inputs: LauncherIO.LauncherInputs) {
        inputs.hasNote = noteDetector.get()

        inputs.topFlywheel.relativePosition.mut_replace(topFlywheelsEncoder.position, Rotations)
        inputs.topFlywheel.velocity.mut_replace(topFlywheelsEncoder.velocity, RPM)
        inputs.topFlywheel.appliedVoltage.mut_replace(topFlywheels.appliedOutput, Volts)
        inputs.topFlywheel.appliedCurrent.mut_replace(topFlywheels.outputCurrent, Amps)

        inputs.bottomFlywheel.relativePosition.mut_replace(bottomFlywheelsEncoder.position, Rotations)
        inputs.bottomFlywheel.velocity.mut_replace(bottomFlywheelsEncoder.velocity, RPM)
        inputs.bottomFlywheel.appliedVoltage.mut_replace(bottomFlywheels.appliedOutput, Volts)
        inputs.bottomFlywheel.appliedCurrent.mut_replace(bottomFlywheels.outputCurrent, Amps)

        inputs.rollerRelativePosition.mut_replace(rollerEncoder.position, Rotations)
        inputs.rollerVelocity.mut_replace(rollerEncoder.velocity, RPM)
        inputs.rollerAppliedVoltage.mut_replace(roller.appliedOutput, Volts)
        inputs.rollerAppliedCurrent.mut_replace(roller.outputCurrent, Amps)
    }

    override fun setTopFlywheelVoltage(voltage: Measure<Voltage>) {
        topFlywheels.setVoltage(voltage into Volts)
    }

    override fun setBottomFlywheelVoltage(voltage: Measure<Voltage>) {
        bottomFlywheels.setVoltage(voltage into Volts)
    }

    /**
     * Runs the flywheels at a specified velocity.
     *
     * @param velocity The velocity to run the flywheel motor(s) at.
     */
    override fun setFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {
        val setpoint = velocity into RPM
        topFlywheelsPIDController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                topFlywheelsFeedForward.calculate(setpoint)
        )
        bottomFlywheelsPIDController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                bottomFlywheelsFeedForward.calculate(setpoint)
        )
    }

    override fun setTopFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {
        val setpoint = velocity into RPM
        topFlywheelsPIDController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                topFlywheelsFeedForward.calculate(setpoint)
        )
    }

    override fun setBottomFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {
        val setpoint = velocity into RPM
        bottomFlywheelsPIDController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                bottomFlywheelsFeedForward.calculate(setpoint)
        )
    }

    override fun setFlywheelLinearVelocity(velocity: Measure<Velocity<Distance>>) {
        // Convert from linear to angular
        setFlywheelVelocity(((velocity into MetersPerSecond) / (flywheelRadius into Meters)).radiansPerSecond)
    }

    /**
     * Runs the roller with the specified voltage.
     *
     * @param voltage The voltage to control the roller motor(s) with.
     */
    override fun setRollerVoltage(voltage: Measure<Voltage>) {
        roller.setVoltage(voltage into Volts)
    }

    /**
     * Runs the roller at a specified velocity.
     *
     * @param velocity The velocity to run the roller motor(s) at.
     */
    override fun setRollerVelocity(velocity: Measure<Velocity<Angle>>) {
        val setpoint = velocity into RPM
        rollerPIDController.setReference(
                setpoint,
                CANSparkBase.ControlType.kVelocity,
                0,
                rollerFeedForward.calculate(setpoint)
        )
    }

    override fun setRollerPosition(position: Measure<Angle>) {
        rollerPIDController.setReference(position into Rotations, CANSparkBase.ControlType.kPosition)
    }

    /**
     * Stops the roller.
     */
    override fun stopRoller() {
        roller.stopMotor()
    }

    /**
     * Stops the flywheels
     */
    override fun stopFlywheels() {
        topFlywheels.stopMotor()
        bottomFlywheels.stopMotor()
    }

    override fun setTopFlywheelPID(p: Double, i: Double, d: Double) {
        topFlywheelsPIDController.p = p
        topFlywheelsPIDController.i = i
        topFlywheelsPIDController.d = d
    }

    override fun setBottomFlywheelPID(p: Double, i: Double, d: Double) {
        bottomFlywheelsPIDController.p = p
        bottomFlywheelsPIDController.i = i
        bottomFlywheelsPIDController.d = d
    }

    override fun setTopFlywheelFeedForward(s: Double, v: Double, a: Double) {
        topFlywheelsFeedForward = SimpleMotorFeedforward(s, v, a)
    }

    override fun setBottomFlywheelFeedForward(s: Double, v: Double, a: Double) {
        bottomFlywheelsFeedForward = SimpleMotorFeedforward(s, v, a)
    }

    /**
     * Configures the PID controller for the roller motor(s)
     *
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    override fun setRollerPID(p: Double, i: Double, d: Double) {
        rollerPIDController.p = p
        rollerPIDController.i = i
        rollerPIDController.d = d
    }

    /**
     * Configures the feed forward for the roller motor(s)
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    override fun setRollerFeedForward(s: Double, v: Double, a: Double) {
        rollerFeedForward = SimpleMotorFeedforward(s, v, a)
    }
}
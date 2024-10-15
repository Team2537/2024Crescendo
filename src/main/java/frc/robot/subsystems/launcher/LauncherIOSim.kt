package frc.robot.subsystems.launcher

import edu.wpi.first.hal.SimBoolean
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import lib.ControllerGains
import lib.math.units.KilogramMetersSquared
import lib.math.units.RotationalInertia
import lib.math.units.into
import lib.math.units.radiansPerSecond

class LauncherIOSim(
    private val topMotor: DCMotor,
    private val topGearing: Double,
    private val topMoiKgm2: Measure<RotationalInertia>,
    private val topGains: ControllerGains,

    private val bottomMotor: DCMotor,
    private val bottomGearing: Double,
    private val bottomMoiKgm2: Measure<RotationalInertia>,
    private val bottomGains: ControllerGains,

    private val rollerMotor: DCMotor,
    private val rollerGearing: Double,
    private val rollerMoiKgm2: Measure<RotationalInertia>,
    private val rollerGains: ControllerGains,

    private val flywheelRadius: Measure<Distance>,
) : LauncherIO {

    private class ClosedLoopDCMotorSim(
        motor: DCMotor,
        gearing: Double,
        moi: Measure<RotationalInertia>,
        gains: ControllerGains,
    ) : DCMotorSim(motor, gearing, moi into KilogramMetersSquared) {

        val pidController: PIDController = PIDController(gains.kP, gains.kI, gains.kD)
        var feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(gains.kS, gains.kV, gains.kA)
            private set

        // cache for logging
        val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** Whether to run with pid or not */
        var runClosed: Boolean = false
        var setpointIsVelocity: Boolean = false

        fun setInputVoltage(voltage: Measure<Voltage>) {
            super.setInputVoltage(voltage into Volts)
            cachedVoltage.mut_replace(voltage)
        }

        override fun setInputVoltage(volts: Double) {
            super.setInputVoltage(volts)
            cachedVoltage.mut_replace(volts, Volts)
        }

        fun setFeedforward(s: Double, v: Double, a: Double) {
            feedforward = SimpleMotorFeedforward(s, v, a)
        }
    }

    private class ClosedLoopFlywheelSim(
        motor: DCMotor,
        gearing: Double,
        moi: Measure<RotationalInertia>,
        gains: ControllerGains,
    ) : FlywheelSim(motor, gearing, moi into KilogramMetersSquared) {
        val pidController: PIDController = PIDController(gains.kP, gains.kI, gains.kD)
        var feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(gains.kS, gains.kV, gains.kA)
            private set

        val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** Whether to run with pid or not */
        var runClosed: Boolean = false

        fun setInputVoltage(voltage: Measure<Voltage>) {
            super.setInputVoltage(voltage into Volts)
            cachedVoltage.mut_replace(voltage)
        }

        override fun setInputVoltage(volts: Double) {
            super.setInputVoltage(volts)
            cachedVoltage.mut_replace(volts, Volts)
        }

        fun setFeedforward(s: Double, v: Double, a: Double) {
            feedforward = SimpleMotorFeedforward(s, v, a)
        }
    }

    private val topFlywheelSim: ClosedLoopFlywheelSim = ClosedLoopFlywheelSim(
        topMotor,
        topGearing,
        topMoiKgm2,
        topGains,
    )

    private val bottomFlywheelSim: ClosedLoopFlywheelSim = ClosedLoopFlywheelSim(
        bottomMotor,
        bottomGearing,
        bottomMoiKgm2,
        bottomGains
    )

    private val rollerSim: ClosedLoopDCMotorSim = ClosedLoopDCMotorSim(
        rollerMotor,
        rollerGearing,
        rollerMoiKgm2,
        rollerGains
    )

    override fun updateInputs(inputs: LauncherIO.LauncherInputs) {
        inputs.hasNote = SmartDashboard.getBoolean("sim/launcher/noteDetector", false)

        inputs.topFlywheel.run {
            topFlywheelSim.run {
                velocity.mut_replace(angularVelocityRPM, RPM)
                appliedCurrent.mut_replace(currentDrawAmps, Amps)
                appliedVoltage.mut_replace(cachedVoltage)

                if (runClosed)
                    setInputVoltage(pidController.calculate(angularVelocityRadPerSec) + feedforward.calculate(pidController.setpoint, 0.0))
                update(0.02) // 20ms
            }
        }

        inputs.bottomFlywheel.run {
            bottomFlywheelSim.run {
                velocity.mut_replace(angularVelocityRPM, RPM)
                appliedCurrent.mut_replace(currentDrawAmps, Amps)
                appliedVoltage.mut_replace(cachedVoltage)

                if (runClosed)
                    setInputVoltage(pidController.calculate(angularVelocityRadPerSec) + feedforward.calculate(pidController.setpoint, 0.0))
                update(0.02) // 20ms
            }
        }

        inputs.rollerVelocity.mut_replace(rollerSim.angularVelocityRPM, RPM)
        inputs.rollerRelativePosition.mut_replace(rollerSim.angularPositionRotations, Rotations)
        inputs.rollerAppliedCurrent.mut_replace(rollerSim.currentDrawAmps, Amps)
        inputs.rollerAppliedVoltage.mut_replace(rollerSim.cachedVoltage)

        if (rollerSim.runClosed) {
            if(rollerSim.setpointIsVelocity)
                rollerSim.run { setInputVoltage(pidController.calculate(angularVelocityRadPerSec) + feedforward.calculate(pidController.setpoint, 0.0)) }
            else
                rollerSim.run { setInputVoltage(pidController.calculate(angularPositionRad)) }
        }
        rollerSim.update(0.02) // 20ms
    }

    override fun setTopFlywheelVoltage(voltage: Measure<Voltage>) {
        topFlywheelSim.runClosed = false
        topFlywheelSim.setInputVoltage(voltage)
    }

    override fun setBottomFlywheelVoltage(voltage: Measure<Voltage>) {
        bottomFlywheelSim.runClosed = false
        bottomFlywheelSim.setInputVoltage(voltage)
    }

    override fun setTopFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {
        topFlywheelSim.runClosed = true
        topFlywheelSim.pidController.setpoint = velocity into RadiansPerSecond
    }

    override fun setBottomFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomFlywheelSim.runClosed = true
        bottomFlywheelSim.pidController.setpoint = velocity into RadiansPerSecond
    }

    override fun setFlywheelLinearVelocity(velocity: Measure<Velocity<Distance>>) {
        // Convert from linear to angular
        setFlywheelVelocity(((velocity into MetersPerSecond) / (flywheelRadius into Meters)).radiansPerSecond)
    }

    override fun setRollerVoltage(voltage: Measure<Voltage>) {
        rollerSim.runClosed = false
        rollerSim.setInputVoltage(voltage)
    }

    override fun setRollerVelocity(velocity: Measure<Velocity<Angle>>) {
        rollerSim.runClosed = true
        rollerSim.setpointIsVelocity = true
        rollerSim.pidController.setpoint = velocity into RadiansPerSecond
    }


    override fun setRollerPosition(position: Measure<Angle>) {
        rollerSim.runClosed = true
        rollerSim.setpointIsVelocity = false
        rollerSim.pidController.setpoint = position into Radians
    }

    override fun stopRoller() {
        rollerSim.runClosed = false
        rollerSim.setInputVoltage(0.0)
    }

    override fun stopFlywheels() {
        topFlywheelSim.runClosed = false
        topFlywheelSim.setInputVoltage(0.0)
        bottomFlywheelSim.runClosed = false
        bottomFlywheelSim.setInputVoltage(0.0)
    }

    override fun setTopFlywheelPID(p: Double, i: Double, d: Double) {
        topFlywheelSim.pidController.run {
            this.p = p
            this.i = i
            this.d = d
        }
    }

    override fun setBottomFlywheelPID(p: Double, i: Double, d: Double) {
        bottomFlywheelSim.pidController.run {
            this.p = p
            this.i = i
            this.d = d
        }
    }

    override fun setTopFlywheelFeedForward(s: Double, v: Double, a: Double) {
        topFlywheelSim.setFeedforward(s, v, a)
    }

    override fun setBottomFlywheelFeedForward(s: Double, v: Double, a: Double) {
        bottomFlywheelSim.setFeedforward(s, v, a)
    }

    override fun setRollerPID(p: Double, i: Double, d: Double) {
        rollerSim.pidController.run {
            this.p = p
            this.i = i
            this.d = d
        }
    }

    override fun setRollerFeedForward(s: Double, v: Double, a: Double) {
        rollerSim.setFeedforward(s, v, a)
    }

    init {
        if(isSimulation()){
            SmartDashboard.putBoolean("sim/launcher/noteDetector", false)
        }
    }
}

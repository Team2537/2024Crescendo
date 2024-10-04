package frc.robot.subsystems.launcher

import edu.wpi.first.hal.SimBoolean
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import lib.math.units.KilogramMetersSquared
import lib.math.units.RotationalInertia
import lib.math.units.into
import lib.math.units.radiansPerSecond

class LauncherIOSim private constructor(
        topMotor: DCMotor,
        topGearing: Double,
        topMoiKgm2: Double,
        topP: Double, topI: Double, topD: Double,
        topS: Double, topV: Double, topA: Double,
        bottomMotor: DCMotor,
        bottomGearing: Double,
        bottomMoiKgm2: Double,
        bottomP: Double, bottomI: Double, bottomD: Double,
        bottomS: Double, bottomV: Double, bottomA: Double,
        rollerMotor: DCMotor,
        rollerGearing: Double,
        rollerMoiKgm2: Double,
        rollerP: Double, rollerI: Double, rollerD: Double,
        rollerS: Double, rollerV: Double, rollerA: Double,
        noteDetectorId: Int,
        private val flywheelRadiusMeters: Double,
) : LauncherIO {
    class SimNotConfiguredException(msg: String) : Exception(msg)

    class LauncherIOSimBuilder {
        private var topMotor: DCMotor? = null
        private var topGearing: Double = 0.0
        private var topMoiKgm2: Double = 0.0
        private var topP: Double = 0.0
        private var topI: Double = 0.0
        private var topD: Double = 0.0
        private var topS: Double = 0.0
        private var topV: Double = 0.0
        private var topA: Double = 0.0

        private var bottomMotor: DCMotor? = null
        private var bottomGearing: Double = 0.0
        private var bottomMoiKgm2: Double = 0.0
        private var bottomP: Double = 0.0
        private var bottomI: Double = 0.0
        private var bottomD: Double = 0.0
        private var bottomS: Double = 0.0
        private var bottomV: Double = 0.0
        private var bottomA: Double = 0.0

        private var rollerMotor: DCMotor? = null
        private var rollerGearing: Double = 0.0
        private var rollerMoiKgm2: Double = 0.0
        private var rollerP: Double = 0.0
        private var rollerI: Double = 0.0
        private var rollerD: Double = 0.0
        private var rollerS: Double = 0.0
        private var rollerV: Double = 0.0
        private var rollerA: Double = 0.0

        private var noteDetectorId: Int = 0

        private var flywheelRadiusMeters: Double = 0.0

        fun configureTopFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.topMotor = motor
            this.topGearing = gearing
            this.topMoiKgm2 = moi into KilogramMetersSquared
            return this
        }

        fun configureTopFlywheelPID(p: Double, i: Double, d: Double): LauncherIOSimBuilder {
            topP = p
            topI = i
            topD = d
            return this
        }

        fun configureTopFlywheelFeedforward(s: Double, v: Double, a: Double): LauncherIOSimBuilder {
            topS = s
            topV = v
            topA = a
            return this
        }

        fun configureBottomFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.bottomMotor = motor
            this.bottomGearing = gearing
            this.bottomMoiKgm2 = moi into KilogramMetersSquared
            return this
        }

        fun configureBottomFlywheelPID(p: Double, i: Double, d: Double): LauncherIOSimBuilder {
            bottomP = p
            bottomI = i
            bottomD = d
            return this
        }

        fun configureBottomFlywheelFeedforward(s: Double, v: Double, a: Double): LauncherIOSimBuilder {
            bottomS = s
            bottomV = v
            bottomA = a
            return this
        }

        fun configureFlywheelRadius(radius: Measure<Distance>) {
            flywheelRadiusMeters = radius into Meters
        }

        fun configureRoller(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.rollerMotor = motor
            this.rollerGearing = gearing
            this.rollerMoiKgm2 = moi into KilogramMetersSquared
            return this
        }

        fun configureRollerPID(p: Double, i: Double, d: Double): LauncherIOSimBuilder {
            rollerP = p
            rollerI = i
            rollerD = d
            return this
        }

        fun configureRollerFeedforward(s: Double, v: Double, a: Double): LauncherIOSimBuilder {
            rollerS = s
            rollerV = v
            rollerA = a
            return this
        }

        fun configureNoteDetector(id: Int) {
            this.noteDetectorId = id
        }

        /**
         * Build the configured sim
         *
         * @return A configured [LauncherIOSim].
         *
         * @throws SimNotConfiguredException If the sim is fatally missing configurations, such as motors.
         */
        @Throws(SimNotConfiguredException::class)
        fun build(): LauncherIOSim {
            if(topMotor === null || bottomMotor === null || rollerMotor === null) {
                throw SimNotConfiguredException(missingMotorsMessage())
            }
            return LauncherIOSim(
                    topMotor!!,
                    topGearing,
                    topMoiKgm2,
                    topP, topI, topD,
                    topS, topV, topA,
                    bottomMotor!!,
                    bottomGearing,
                    bottomMoiKgm2,
                    bottomP, bottomI, bottomD,
                    bottomS, bottomV, bottomA,
                    rollerMotor!!,
                    rollerGearing,
                    rollerMoiKgm2,
                    rollerP, rollerI, rollerD,
                    rollerS, rollerV, rollerA,
                    noteDetectorId,
                    flywheelRadiusMeters,
            )
        }

        /**
         * Build the configured sim
         *
         * @param onFail A fail condition; called if the sim cannot be built.
         *
         * @return A configured [LauncherIOSim], or `null` if the build failed.
         */
        fun build(onFail: (SimNotConfiguredException) -> Unit): LauncherIOSim? {
            try {
                return build()
            } catch (e: SimNotConfiguredException) {
                onFail.invoke(e)
            }
            return null
        }

        private fun missingMotorsMessage(): String {
            val msg = StringBuilder()
            var i = 0

            if(topMotor === null) {
                i++
                msg.append(" topFlywheelMotor")
            }
            if(bottomMotor === null) {
                i++
                msg.append(" bottomFlywheelMotor")
            }
            if(rollerMotor === null) {
                i++
                msg.append(" rollerMotor")
            }

            return if(i > 1)
                "Motor not configured:${msg}"
            else
                "Motors not configured:${msg}"
        }
    }

    private class ClosedLoopDCMotorSim(
        motor: DCMotor,
        gearing: Double,
        moi: Double,
        p: Double, i: Double, d: Double,
        s: Double, v: Double, a: Double,
    ) : DCMotorSim(motor, gearing, moi) {

        val pidController: PIDController = PIDController(p, i, d)
        var feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(s, v, a)
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
        moi: Double,
        p: Double, i: Double, d: Double,
        s: Double, v: Double, a: Double,
    ) : FlywheelSim(motor, gearing, moi) {
        val pidController: PIDController = PIDController(p, i, d)
        var feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(s, v, a)
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
        topP, topI, topD,
        topS, topV, topA,
    )

    private val bottomFlywheelSim: ClosedLoopFlywheelSim = ClosedLoopFlywheelSim(
        bottomMotor,
        bottomGearing,
        bottomMoiKgm2,
        bottomP, bottomI, bottomD,
        bottomS, bottomV, bottomA,
    )

    private val rollerSim: ClosedLoopDCMotorSim = ClosedLoopDCMotorSim(
        rollerMotor,
        rollerGearing,
        rollerMoiKgm2,
        rollerP, rollerI, rollerD,
        rollerS, rollerV, rollerA,
    )

    private val noteDetector: SimBoolean = SimBoolean(noteDetectorId)

    /**
     * Initial builder for Launcher sim. Calling any method will instantiate an instance of
     * [LauncherIOSimBuilder] for you.
     */
    companion object {
        @JvmStatic
        fun configureTopFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            return LauncherIOSimBuilder().configureTopFlywheel(motor, gearing, moi)
        }

        @JvmStatic
        fun configureBottomFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            return LauncherIOSimBuilder().configureBottomFlywheel(motor, gearing, moi)
        }

        @JvmStatic
        fun configureRoller(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            return LauncherIOSimBuilder().configureRoller(motor, gearing, moi)
        }
    }

    override fun updateInputs(inputs: LauncherIO.LauncherInputs) {
        inputs.hasNote = noteDetector.get()

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
        setFlywheelVelocity(((velocity into MetersPerSecond) / flywheelRadiusMeters).radiansPerSecond)
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
}

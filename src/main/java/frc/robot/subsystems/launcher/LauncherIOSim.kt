package frc.robot.subsystems.launcher

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import lib.math.units.KilogramMetersSquared
import lib.math.units.RotationalInertia
import lib.math.units.into

class LauncherIOSim private constructor(
        topMotor: DCMotor,
        topGearing: Double,
        topMoiKgm2: Double,
        bottomMotor: DCMotor,
        bottomGearing: Double,
        bottomMoiKgm2: Double,
        rollerMotor: DCMotor,
        rollerGearing: Double,
        rollerMoiKgm2: Double,
) : LauncherIO {
    class SimNotConfiguredException(msg: String) : Exception(msg)

    class LauncherIOSimBuilder {
        private var topMotor: DCMotor? = null
        private var topGearing: Double = 0.0
        private var topMoiKgm2: Double = 0.0

        private var bottomMotor: DCMotor? = null
        private var bottomGearing: Double = 0.0
        private var bottomMoiKgm2: Double = 0.0

        private var rollerMotor: DCMotor? = null
        private var rollerGearing: Double = 0.0
        private var rollerMoiKgm2: Double = 0.0

        fun configureTopFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.topMotor = motor
            this.topGearing = gearing
            this.topMoiKgm2 = moi into KilogramMetersSquared
            return this
        }

        fun configureBottomFlywheel(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.bottomMotor = motor
            this.bottomGearing = gearing
            this.bottomMoiKgm2 = moi into KilogramMetersSquared
            return this
        }

        fun configureRoller(motor: DCMotor, gearing: Double, moi: Measure<RotationalInertia>): LauncherIOSimBuilder {
            this.rollerMotor = motor
            this.rollerGearing = gearing
            this.rollerMoiKgm2 = moi into KilogramMetersSquared
            return this
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
                    bottomMotor!!,
                    bottomGearing,
                    bottomMoiKgm2,
                    rollerMotor!!,
                    rollerGearing,
                    rollerMoiKgm2,
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

    private val topFlywheelSim: FlywheelSim

    private val bottomFlywheelSim: FlywheelSim

    private val rollerSim: DCMotorSim

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

    init {
        topFlywheelSim = FlywheelSim(
                topMotor,
                topGearing,
                topMoiKgm2,
        )

        bottomFlywheelSim = FlywheelSim(
                bottomMotor,
                bottomGearing,
                bottomMoiKgm2,
        )

        rollerSim = DCMotorSim(
                rollerMotor,
                rollerGearing,
                rollerMoiKgm2,
        )
    }
}

package frc.robot.subsystems.pivot

import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import lib.math.units.into

class PivotIOSim(
    private val motor: DCMotor,
    private val gearing: Double,
    private val moi: Double,
    private val armLength: Measure<Distance>,
    private val maxAngle: Measure<Angle>,
    private val minAngle: Measure<Angle>,
    private val startAngle: Measure<Angle>,
    private val kP: Double, private val kI: Double, private val kD: Double,
    private val kS: Double, private val kG: Double, private val kV: Double, private val kA: Double,
) : PivotIO {
    /** Simulation class for the pivot arm. */
    private val sim: SingleJointedArmSim = SingleJointedArmSim(
        motor,
        gearing,
        moi,
        armLength into Meters,
        maxAngle into Radians,
        minAngle into Radians,
        true,
        startAngle into Radians
    )

    /** PID controller for the pivot arm. */
    private val pid: PIDController = PIDController(kP, kI, kD)
    /** Feedforward controller for the pivot arm. */
    private val feedforward: ArmFeedforward = ArmFeedforward(kS, kG, kV, kA)

    /** Voltage being applied to the motor, stored for logging purposes. */
    private val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
    /** Whether the voltage is being set by a PID controller. */
    private var isPID = false


    /**
     * Updates the inputs with new data.
     * @param inputs The data to update with the new sensor information, mutated in place.
     */
    override fun updateInputs(inputs: PivotIO.PivotInputs) {
        inputs.isAtHardstop = sim.hasHitUpperLimit()

        inputs.relativePosition.mut_replace(sim.angleRads, Radians)
        inputs.absolutePosition.mut_replace(sim.angleRads, Radians)
        inputs.velocity.mut_replace(sim.velocityRadPerSec, RadiansPerSecond)
        inputs.appliedVoltage.mut_replace(cachedVoltage)
        inputs.appliedCurrent.mut_replace(Amps.of(sim.currentDrawAmps))

        if(isPID){
            setRawVoltage(Volts.of(pid.calculate(sim.angleRads) + feedforward.calculate(pid.setpoint, 0.0)), true)
        }

        sim.update(0.02)
    }

    /**
     * Sets the raw voltage applied to the motor(s).
     * @param voltage The voltage to apply.
     * @param isPID Whether the voltage is being set by a PID controller. (Used for simulation purposes.)
     */
    override fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean) {
        cachedVoltage.mut_replace(voltage)
        sim.setInputVoltage(voltage into Volts)
        this.isPID = isPID
    }

    /**
     * Reset the relative encoder to a known position.
     * @param position The known position to reset to.
     */
    override fun setKnownPosition(position: Measure<Angle>) {
        sim.setState(position into Radians, 0.0)
    }

    /**
     * Set the target position of the pivot.
     * @param position The target position to set.
     */
    override fun setTargetPosition(position: Measure<Angle>) {
        pid.setpoint = position into Radians
    }

    /**
     * Set the PID gains of the pivot.
     * @param p The proportional gain.
     * @param i The integral gain.
     * @param d The derivative gain.
     */
    override fun setPID(p: Double, i: Double, d: Double) {
        pid.p = p
        pid.i = i
        pid.d = d
    }
}
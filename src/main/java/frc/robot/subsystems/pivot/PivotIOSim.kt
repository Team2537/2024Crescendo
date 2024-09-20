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
import edu.wpi.first.units.Units.Radian
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

    private val pid: PIDController = PIDController(kP, kI, kD)
    private val feedforward: ArmFeedforward = ArmFeedforward(kS, kG, kV, kA)

    private val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
    private var isPID = false


    override fun updateInputs(inputs: PivotIO.PivotInputs) {
        inputs.isAtHardstop = sim.hasHitUpperLimit()

        inputs.pivotRelativePosition.mut_replace(sim.angleRads, Radians)
        inputs.pivotAbsolutePosition.mut_replace(sim.angleRads, Radians)
        inputs.pivotAngularVelocity.mut_replace(sim.velocityRadPerSec, RadiansPerSecond)
        inputs.pivotMotorAppliedVoltage.mut_replace(cachedVoltage)
        inputs.pivotMotorAppliedCurrent.mut_replace(Amps.of(sim.currentDrawAmps))

        if(isPID){
            setRawVoltage(Volts.of(pid.calculate(sim.angleRads) + feedforward.calculate(pid.setpoint, 0.0)), true)
        }

        sim.update(0.02)
    }

    override fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean) {
        cachedVoltage.mut_replace(voltage)
        sim.setInputVoltage(voltage into Volts)
        this.isPID = isPID
    }

    override fun setKnownPosition(position: Measure<Angle>) {
        sim.setState(position into Radians, 0.0)
    }

    override fun setTargetPosition(position: Measure<Angle>) {
        pid.setpoint = position into Radians
    }

    override fun setPID(p: Double, i: Double, d: Double) {
        pid.p = p
        pid.i = i
        pid.d = d
    }
}
package frc.robot.subsystems.launcher.flywheels

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import lib.ControllerGains
import lib.math.units.into

class FlywheelIOSim(
    private val motor: DCMotor,
    private val moiKgM2: Double,
    private val gains: ControllerGains
) : FlywheelIO {
    private val sim: DCMotorSim = DCMotorSim(motor, 1.0, moiKgM2)

    private val feedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        gains.kS,
        gains.kV,
        gains.kA
    )

    private val pid: PIDController = PIDController(
        gains.kP,
        gains.kI,
        gains.kD
    )

    private val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
    private var isClosedLoop = false

    override fun updateInputs(inputs: FlywheelIO.FlywheelInputs) {
        sim.update(0.02)

        inputs.velocity.mut_replace(sim.angularVelocityRPM, RPM)
        inputs.position.mut_replace(sim.angularPositionRotations, Rotations)
        inputs.statorCurrent.mut_replace(sim.currentDrawAmps, Amps)
        inputs.motorVoltage.mut_replace(cachedVoltage)
        inputs.supplyVoltage.mut_replace(RobotController.getBatteryVoltage(), Volts)

        if(isClosedLoop){
            sim.setInputVoltage(pid.calculate(inputs.velocity into RPM))
        }
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        this.isClosedLoop = isClosedLoop
        cachedVoltage.mut_replace(voltage)
        sim.setInputVoltage(voltage into Volts)
    }

    override fun setVelocity(velocity: Measure<Velocity<Angle>>) {
        isClosedLoop = true
        pid.setpoint = velocity into RPM
    }

    override fun setBrakeMode(isBrakeMode: Boolean) {
        super.setBrakeMode(isBrakeMode)
    }
}
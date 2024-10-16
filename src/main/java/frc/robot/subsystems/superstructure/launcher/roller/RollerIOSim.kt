package frc.robot.subsystems.superstructure.launcher.roller

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import lib.ControllerGains
import lib.math.units.Rotation
import lib.math.units.into

class RollerIOSim(
    private val motor: DCMotor,
    private val moiKgM2: Double,
    private val rollerRadius: Measure<Distance>,
    private val gains: ControllerGains
) : RollerIO {
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

    init {
        SmartDashboard.putBoolean("sim/roller/noteDetected", false)
    }

    private val cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)
    private var isClosedLoop: Boolean = false

    override fun updateInputs(inputs: RollerIO.RollerInputs) {
        sim.update(0.02)

        inputs.position.mut_replace(sim.angularPositionRotations, Rotations)
        inputs.velocity.mut_replace(sim.angularVelocityRPM, RPM)
        inputs.statorCurrent.mut_replace(sim.currentDrawAmps, Amps)
        inputs.motorVoltage.mut_replace(cachedVoltage)
        inputs.supplyVoltage.mut_replace(RobotController.getBatteryVoltage(), Volts)
        inputs.noteDetected = SmartDashboard.getBoolean("sim/roller/noteDetected", false)

        if(isClosedLoop) {
            setVoltage(
                Volts.of(
                    pid.calculate(sim.angularPositionRotations)
                ),
                true
            )
        }
    }

    override fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean) {
        cachedVoltage.mut_replace(voltage)
        this.isClosedLoop = isClosedLoop
        sim.setInputVoltage(voltage into Volts)
    }

    override fun setTargetPosition(position: Measure<Distance>) {
        // Convert the target position to rotations
        val targetRotations = Units.radiansToRotations((position into Meters) / (rollerRadius into Meters))
        pid.setpoint = targetRotations
        isClosedLoop = true
    }
}
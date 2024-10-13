package frc.robot.subsystems.intake

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Meter
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.robot.Robot
import lib.GamepieceFieldSimulator
import lib.math.units.into
import lib.toTransform2d
import java.util.function.Supplier

class IntakeIOSim(
    gearing: Double,
    moi: Double,
    private val rollerDiameter: Measure<Distance>,
    private val state: Supplier<Intake.IntakeState>
) : IntakeIO {
    private val intakeSim: FlywheelSim = FlywheelSim(
        DCMotor.getNeo550(1),
        gearing,
        moi
    )

    private var noteJustIntook = false
    private var noteJustExited = false


    private val fieldSim: GamepieceFieldSimulator = GamepieceFieldSimulator(
        GamepieceFieldSimulator.FRCGameField.CRESCENDO,
        { Robot.robotPose.transformBy(Intake.poseToIntakePoint.toTransform2d()) },
        { (intakeSim.angularVelocityRadPerSec * ((rollerDiameter into Meters) / 2)) > 6.0 },
        { noteJustIntook = true }
    )

    private var cachedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

    override fun updateInputs(inputs: IntakeIO.IntakeInputs) {
        intakeSim.update(0.02)
        fieldSim.update()

        noteJustExited = (intakeSim.angularVelocityRadPerSec * ((rollerDiameter into Meters) / 2)) < -6.0 && !noteJustExited

        inputs.intakeSensorTriggered =
            noteJustIntook || noteJustExited
        inputs.exitSensorTriggered =
            (intakeSim.angularVelocityRadPerSec * ((rollerDiameter into Meters) / 2)) > 6.0 && state.get() == Intake.IntakeState.STORED
        inputs.linearVelocity
            .mut_replace(
                intakeSim.angularVelocityRadPerSec * ((rollerDiameter into Meter) / 2),
                MetersPerSecond
            ) // Convert from rad/s to m/s
        inputs.supplyVoltage.mut_replace(RobotController.getBatteryVoltage(), Volts)
        inputs.motorVoltage.mut_replace(cachedVoltage)
        inputs.statorCurrent.mut_replace(intakeSim.currentDrawAmps, Amps)

        if(noteJustIntook) noteJustIntook = false
    }

    override fun setVoltage(voltage: Measure<Voltage>) {
        cachedVoltage.mut_replace(voltage)
        intakeSim.setInputVoltage(voltage into Volts)
    }

    override fun stop() {
        setVoltage(Volts.of(0.0))
    }
}
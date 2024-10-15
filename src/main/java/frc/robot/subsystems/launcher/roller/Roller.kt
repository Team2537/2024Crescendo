package frc.robot.subsystems.launcher.roller

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Constants.RobotConstants.mode
import lib.ControllerGains
import lib.math.units.into
import lib.math.units.measuredIn
import lib.not
import org.littletonrobotics.junction.Logger

class Roller : SubsystemBase("roller") {
    val rollerIO: RollerIO = when(mode){
        Constants.RobotConstants.Mode.REAL -> RollerIONeo(
            motorID,
            noteDetectorID,
            isInverted,
            ControllerGains(),
            rollerRadius = rollerRadius
        )
        Constants.RobotConstants.Mode.SIM -> RollerIOSim(
            motor = DCMotor.getNEO(1),
            moiKgM2 = moiKgM2,
            rollerRadius = rollerRadius,
            gains = ControllerGains()
        )
        Constants.RobotConstants.Mode.REPLAY -> TODO()
    }

    val inputs: RollerIO.RollerInputs = RollerIO.RollerInputs()

    val positionLinear: Measure<Distance>
        get() = ((inputs.position into Radians) * (rollerRadius into Meters)) measuredIn Meters

    val setpoint: MutableMeasure<Distance> = MutableMeasure.zero(Inches)

    val isAtSetpoint: Trigger = Trigger { inputs.position.isNear(setpoint, 0.01) }.debounce(0.2)
    val isHoldingNote: Trigger = Trigger { inputs.noteDetected }.debounce(0.2)

    override fun periodic() {
        rollerIO.updateInputs(inputs)
        Logger.processInputs("launcher/roller", inputs)

        Logger.recordOutput("launcher/roller/linearPosition", positionLinear)
        Logger.recordOutput("launcher/roller/setpoint", setpoint)
    }

    fun getPullNoteCommand(distance: Measure<Distance> = Inches.of(6.0)) =
        Commands.sequence(
            runOnce {
                rollerIO.setBrakeMode(true)
                setpoint.mut_replace(positionLinear + distance)
                rollerIO.setTargetPosition(setpoint)
            },
            waitUntil(isAtSetpoint),
        )

    fun getPushNoteCommand() =
        Commands.sequence(
            runOnce {
                rollerIO.setBrakeMode(false)
                rollerIO.setVoltage(Volts.of(12.0))
            },
            waitUntil(!isHoldingNote),
            runOnce { rollerIO.setVoltage(Volts.zero()) }
        )

    companion object {
        val motorID: Int = 100 // TODO: Set the id
        val noteDetectorID: Int = 200 // TODO: Set the id
        val isInverted: Boolean = false // TODO: Set the isInverted
        val rollerRadius = Inches.of(1.460)
        val moiKgM2 = 0.0000434119
    }
}


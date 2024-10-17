package frc.robot.subsystems.superstructure.launcher.roller

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.waitSeconds
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import frc.robot.Constants.RobotConstants.mode
import lib.ControllerGains
import lib.LoggedTunableNumber
import lib.math.units.into
import lib.math.units.measuredIn
import lib.not
import org.littletonrobotics.junction.Logger

class Roller : SubsystemBase("roller") {

    private val kP = LoggedTunableNumber("roller/kP", 0.5)
    private val kI = LoggedTunableNumber("roller/kI", 0.0)
    private val kD = LoggedTunableNumber("roller/kD", 0.0)

    val rollerIO: RollerIO = when(mode){
        Constants.RobotConstants.Mode.REAL -> RollerIONeo(
            motorID,
            noteDetectorID,
            isInverted,
            ControllerGains(kP = kP.get(), kI = kI.get(), kD = kD.get()),
            rollerRadius = rollerRadius
        )
        Constants.RobotConstants.Mode.SIM -> RollerIOSim(
            motor = DCMotor.getNEO(1),
            moiKgM2 = moiKgM2,
            rollerRadius = rollerRadius,
            gains = ControllerGains(kP = 0.5)
        )
        Constants.RobotConstants.Mode.REPLAY -> TODO()
    }

    val inputs: RollerIO.RollerInputs = RollerIO.RollerInputs()

    val positionLinear: Measure<Distance>
        get() = ((inputs.position into Radians) * (rollerRadius into Meters)) measuredIn Meters

    val setpoint: MutableMeasure<Distance> = MutableMeasure.zero(Inches)

    val isAtSetpoint: Trigger = Trigger { positionLinear.isNear(setpoint, 0.1) }.debounce(0.2)
    val isHoldingNote: Trigger = Trigger { inputs.noteDetected }.debounce(0.2)

    override fun periodic() {
        rollerIO.updateInputs(inputs)
        Logger.processInputs("launcher/roller", inputs)

        Logger.recordOutput("launcher/roller/linearPosition", positionLinear)
        Logger.recordOutput("launcher/roller/setpoint", setpoint)

        Logger.recordOutput("launcher/roller/isAtSetpoint", isAtSetpoint.asBoolean)
        Logger.recordOutput("launcher/roller/isHoldingNote", isHoldingNote.asBoolean)

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> rollerIO.setPID(pid[0], pid[1], pid[2]) },
            kP, kI, kD
        )
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

    fun getPushNoteCommand(voltage: Measure<Voltage> = Volts.of(12.0)) =
        Commands.sequence(
            runOnce {
                rollerIO.setBrakeMode(false)
                rollerIO.setVoltage(voltage)
            },
            waitUntil(!isHoldingNote),
            getStopCommand()
        )

    fun getStopCommand() =
        runOnce {
            rollerIO.setBrakeMode(true)
            rollerIO.setVoltage(Volts.zero())
        }

    fun getEjectCommand() =
        Commands.sequence(
            runOnce {
                rollerIO.setBrakeMode(false)
                rollerIO.setVoltage(Volts.of(-4.0))
            },
            waitUntil(!isHoldingNote),
            waitSeconds(0.5),
            getStopCommand()
        )


    companion object {
        val motorID: Int = 14
        val noteDetectorID: Int = 0
        val isInverted: Boolean = true
        val rollerRadius = Inches.of(1.460)
        val moiKgM2 = 0.0000434119
    }
}


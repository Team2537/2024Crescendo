package frc.robot.subsystems.superstructure.launcher.flywheels

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.Constants.RobotConstants.mode
import lib.ControllerGains
import lib.LoggedTunableNumber
import org.littletonrobotics.junction.Logger

class Flywheels : SubsystemBase("flywheels") {
    private val topKp = LoggedTunableNumber("flywheels/top/kP", 0.005)
    private val bottomKp = LoggedTunableNumber("flywheels/bottom/kP", 0.005)

    private val topKi = LoggedTunableNumber("flywheels/top/kI", 0.0)
    private val bottomKi = LoggedTunableNumber("flywheels/bottom/kI", 0.0)

    private val topKd = LoggedTunableNumber("flywheels/top/kD", 0.0)
    private val bottomKd = LoggedTunableNumber("flywheels/bottom/kD", 0.0)

    val topFlywheelIO: FlywheelIO = when(mode) {
        Constants.RobotConstants.Mode.REAL -> FlywheelIONeo(
            topID,
            false,
            ControllerGains(
                kP = topKp.get(), kI = topKi.get(), kD = topKd.get(),
                kV = 12.0/7000.0
            )
        )
        Constants.RobotConstants.Mode.SIM -> FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            moiKgM2 = MOI,
            ControllerGains(
                kV = 12.0/6200.0,
                kP = 0.17
            ))
        Constants.RobotConstants.Mode.REPLAY -> object : FlywheelIO {}
    }
    val bottomFlywheelIO: FlywheelIO = when(mode) {
        Constants.RobotConstants.Mode.REAL -> FlywheelIONeo(
            bottomID,
            true,
            ControllerGains(kP = bottomKp.get(), kI = bottomKi.get(), kD = bottomKd.get())
        )
        Constants.RobotConstants.Mode.SIM -> FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            moiKgM2 = MOI,
            ControllerGains(
                kV = 12.0/6900.0,
                kP = 0.17
            ))
        Constants.RobotConstants.Mode.REPLAY -> object : FlywheelIO {}
    }

    val topInputs: FlywheelIO.FlywheelInputs = FlywheelIO.FlywheelInputs()
    val bottomInputs: FlywheelIO.FlywheelInputs = FlywheelIO.FlywheelInputs()

    private val goalVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RPM)

    private val topSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            {state -> Logger.recordOutput("launcher/sysIdState", state.name)}
        ),
        SysIdRoutine.Mechanism(
            {voltage: Measure<Voltage> -> topFlywheelIO.setVoltage(voltage) },
            null,
            this
        )
    )

    private val bottomSysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null,
            {state -> Logger.recordOutput("launcher/sysIdState", state.name)}
        ),
        SysIdRoutine.Mechanism(
            {voltage: Measure<Voltage> -> bottomFlywheelIO.setVoltage(voltage) },
            null,
            this
        )
    )

    private val routineToApply = topSysIdRoutine

    fun getDynamicSysID(direction: Direction) = routineToApply.dynamic(direction)
    fun getQuasistatic(direction: Direction) = routineToApply.quasistatic(direction)

    fun getPrimeCommand(targetVelocity: Measure<Velocity<Angle>>) = runOnce {
        bottomFlywheelIO.setBrakeMode(false)
        topFlywheelIO.setBrakeMode(false)

        goalVelocity.mut_replace(targetVelocity)
        topFlywheelIO.setVelocity(targetVelocity)
        bottomFlywheelIO.setVelocity(targetVelocity)
    }

    fun getRawVoltageCommand(volts: Measure<Voltage>) = run {
        topFlywheelIO.setVoltage(volts)
        bottomFlywheelIO.setVoltage(volts)
    }.finallyDo(Runnable { topFlywheelIO.setVoltage(Volts.zero()); bottomFlywheelIO.setVoltage(Volts.zero())})

    fun getWaitUntilReadyCommand() = Commands.waitUntil {
        topInputs.velocity.isNear(goalVelocity, 0.02) &&
        bottomInputs.velocity.isNear(goalVelocity, 0.02)
    }

    fun getStopCommand() = runOnce {
        topFlywheelIO.setBrakeMode(true)
        bottomFlywheelIO.setBrakeMode(true)

        topFlywheelIO.setVoltage(Volts.zero())
        bottomFlywheelIO.setVoltage(Volts.zero())
    }

    override fun periodic() {
        topFlywheelIO.updateInputs(topInputs)
        bottomFlywheelIO.updateInputs(bottomInputs)

        Logger.processInputs("launcher/top", topInputs)
        Logger.processInputs("launcher/bottom", bottomInputs)

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> topFlywheelIO.setPID(pid[0], pid[1], pid[2])},
            topKp, topKi, topKd
        )

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> bottomFlywheelIO.setPID(pid[0], pid[1], pid[2])},
            bottomKp, bottomKi, bottomKd
        )
    }

    companion object {
        val topID: Int = 22
        val bottomID: Int = 23

        val MOI: Double = 0.001317 * 4
    }
}
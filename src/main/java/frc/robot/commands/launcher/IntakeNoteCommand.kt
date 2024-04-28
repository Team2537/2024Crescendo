package frc.robot.commands.launcher

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.launcher.LaunchSubsystem
import lib.math.units.inches

class IntakeNoteCommand : Command() {
    private val timer: Timer = Timer()
    private val subsystem = LaunchSubsystem
    private val io = subsystem.io
    private val inputs = subsystem.input

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        io.setRollerBrakeMode(false)
        timer.restart()
        io.setRollerPower(0.1)
        io.stopFlywheels()
    }

    override fun execute() {
        io.setRollerPower(-0.1)
        if (!inputs.rightNoteDetected) timer.reset()
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.15) && inputs.rightNoteDetected
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        io.runRollerSetpoint(inputs.rollerPosition + 0.3.inches)
    }
}
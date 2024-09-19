package frc.robot.commands.launcher

import edu.wpi.first.wpilibj2.command.Command
import Launcher
import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

class IntakeNoteCommand(private val subsystem: Launcher) : Command() {
    /** Timer to keep track of how long note has been in the launcher */
    private val timer: Timer = Timer()


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(subsystem)
    }

    /**
     * Start pulling the note into the robot by spinning the roller motor
     */
    override fun initialize() {
        // set roller motor to coast to make it easier to pull in the note
        subsystem.rollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        // Reset the timer so it starts counting from 0 when the command is initialized
        timer.restart()
        // set the roller motor to pull the note in
        subsystem.setRollerSpeed(0.1)
        // stop the flywheels to make sure the note doesn't get launched out
        subsystem.stopFlywheels()
    }

    /**
     * Keep the roller motor spinning to pull the note in
     * If the note trigger is not pressed, reset the timer, so the timer keeps track of how long the note has been in the launcher
     * Runs every 20ms
     */
    override fun execute() {
       subsystem.setRollerSpeed(-0.1)
        SmartDashboard.putNumber("Note Timer", timer.get())
        if(!subsystem.noteTrigger.asBoolean){
            timer.reset()
        }
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(0.15) && subsystem.noteTrigger.asBoolean
    }

    override fun end(interrupted: Boolean) {
        println("Intake note command stopped, Interrupted: $interrupted")
        timer.stop()
        subsystem.setRollerPosition(
            subsystem.getRollerPosition() + 0.3
        )
    }
}

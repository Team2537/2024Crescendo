package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants.IntakeConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import lib.toTrigger

/**
 * The Intake subsystem contains methods and objects for controlling the under-the-bumper intake system,
 * as well as the transfer conveyor(?) to the launcher.
 */
object IntakeSubsystem : SubsystemBase() {
    /**
     * [REV Neo 550](https://www.revrobotics.com/rev-21-1651/) that controls the under-bumper intake.
     */
    val intakeMotor : CANSparkMax = CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless)

    /**
     * Trigger for detecting when a note has entered the intake.
     */
    private val noteDetector: Trigger

    enum class State {
        kNoteHeld,
        kReady,
        kActive,
    }

    private var state: State = State.kReady

    init {
        val pidController = intakeMotor.pidController

        pidController.p = IntakeConstants.INTAKE_KP
        pidController.i = IntakeConstants.INTAKE_KI
        pidController.d = IntakeConstants.INTAKE_KD

        val infraredSensor = DigitalInput(IntakeConstants.INFRARED_SENSOR_CHANNEL)
        noteDetector = infraredSensor.get().toTrigger()

        noteDetector.onTrue(runOnce { state = State.kNoteHeld })
    }

    override fun periodic() {
        updateState()
    }

    private fun updateState() {
        when (state) {
            State.kNoteHeld -> {
                // wait for launch note
            }
            State.kReady -> {
                // wait for detect note (LIMELIGHT YIPPEE)
            }
            State.kActive -> {
                // wait for detect note (INFRARED YIPPEE)
            }
        }
    }

    fun getState(): State = state
}

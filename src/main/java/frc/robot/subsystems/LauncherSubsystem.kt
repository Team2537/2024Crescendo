import com.revrobotics.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Constants
import lib.math.units.RotationVelocity
import lib.math.units.rpm
import lib.math.units.velocity
import lib.near
import lib.zoneTrigger

object LauncherSubsystem : SubsystemBase() {
    // TODO: Get the actual IDs
    val topFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.TOP_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    val bottomFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.BOTTOM_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    val rollerMotor: CANSparkMax = CANSparkMax(Constants.LauncherConstants.ROLLER_MOTOR,
        CANSparkLowLevel.MotorType.kBrushless)

    val leftNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.LEFT_NOTE_DETECTOR)
    val rightNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.RIGHT_NOTE_DETECTOR)

    val tab = Shuffleboard.getTab("Launcher")

    val noteTrigger: Trigger

    init {
        topFlywheels.restoreFactoryDefaults()
        bottomFlywheels.restoreFactoryDefaults()
        rollerMotor.restoreFactoryDefaults()

        topFlywheels.setSmartCurrentLimit(40)
        bottomFlywheels.setSmartCurrentLimit(40)
        rollerMotor.setSmartCurrentLimit(40)

        rollerMotor.pidController.p = 0.0001
        rollerMotor.pidController.i = 0.000001
        rollerMotor.pidController.d = 0.01

        noteTrigger = Trigger() {
            !leftNoteDetector.get() && !rightNoteDetector.get()
        }
    }

    fun setFlywheelSpeeds(rawSpeed: Double) {
        topFlywheels.set(rawSpeed)
    }

    fun setRollerSpeed(rawSpeed: Double) {
        rollerMotor.set(rawSpeed)
    }

    fun stopFlywheels() {
        topFlywheels.set(0.0)
        bottomFlywheels.set(0.0)
    }

    fun stopRoller() {
        rollerMotor.set(0.0)
    }

    fun setRollerPosition(position: Double) {
        rollerMotor.pidController.setReference(position, CANSparkBase.ControlType.kPosition)
    }

    fun getNoteTrigger(): Trigger {
        return noteTrigger
    }

    fun getRollerPosition(): Double {
        return rollerMotor.encoder.position
    }
}
package frc.robot.subsystems.intake

import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.Constants.IntakeConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * The Intake subsystem contains methods and objects for controlling the under-the-bumper intake system,
 * as well as the transfer conveyor(?) to the launcher.
 */
class Intake : SubsystemBase() {
    /** The motor for the intake */
    val intakeMotor : CANSparkMax = CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless)
    /** The motor for the transfer */
    val transferMotor: CANSparkMax = CANSparkMax(IntakeConstants.TRANSFER_MOTOR_ID, MotorType.kBrushless)

    /** Shuffleboard tab for the Intake subsystem */
    private val tab = Shuffleboard.getTab("Intake")

    init {
        // Logging for Shuffleboard, just velocity of the motors
        tab.addDouble("Intake Velocity") { intakeMotor.encoder.velocity }
        tab.addDouble("Transfer Velocity") { transferMotor.encoder.velocity }

        // Set current limits so that the motors don't burn out
        intakeMotor.setSmartCurrentLimit(40)
        transferMotor.setSmartCurrentLimit(30)
    }

    fun feedLauncher() = runEnd({ transferMotor.set(1.0) }, { transferMotor.stopMotor() })
    fun intakeNote() = runEnd({ intakeMotor.set(-1.0); transferMotor.set(1.0) }, { intakeMotor.stopMotor(); transferMotor.stopMotor() })

}

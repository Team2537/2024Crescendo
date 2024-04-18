package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
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

    override fun periodic() {
    }

}

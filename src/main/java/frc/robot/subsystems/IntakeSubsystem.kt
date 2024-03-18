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
    /**
     * [REV Neo 550](https://www.revrobotics.com/rev-21-1651/) that controls the under-bumper intake.
     */
    val intakeMotor : CANSparkMax = CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless)
    val transferMotor: CANSparkMax = CANSparkMax(IntakeConstants.TRANSFER_MOTOR_ID, MotorType.kBrushless)

    private val tab = Shuffleboard.getTab("Intake")

    init {
        tab.addDouble("Intake Velocity") { intakeMotor.encoder.velocity }
        tab.addDouble("Transfer Velocity") { transferMotor.encoder.velocity }

        intakeMotor.setSmartCurrentLimit(40)
        transferMotor.setSmartCurrentLimit(30)
    }

    override fun periodic() {
    }

}

package frc.robot.subsystems

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax
import frc.robot.Constants.IntakeConstants
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.setVelocity

object IntakeSubsystem : SubsystemBase() {
    val intakeMotor : CANSparkMax = CANSparkMax(0, MotorType.kBrushless)

    init {
        val pidController = intakeMotor.pidController

        pidController.p = IntakeConstants.INTAKE_KP
        pidController.i = IntakeConstants.INTAKE_KI
        pidController.d = IntakeConstants.INTAKE_KD
    }
}

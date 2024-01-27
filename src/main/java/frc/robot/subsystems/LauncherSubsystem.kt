package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants

object LauncherSubsystem : SubsystemBase() {
    val angleMotor = CANSparkFlex(LauncherConstants.ANGLE_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val AnglePID: SparkPIDController

    init {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Launcher Subsystem")

        AnglePID = angleMotor.pidController

        AnglePID.p= LauncherConstants.ANGLE_KP
        AnglePID.i= LauncherConstants.ANGLE_KI
        AnglePID.d= LauncherConstants.ANGLE_KD

        driveTab.addNumber("Motor Velocity")
        {
            angleMotor.encoder.velocity
        }

    }

    fun setAngle(angle: Double) {
        AnglePID.setReference(angle, CANSparkBase.ControlType.kPosition)
    }

    override fun periodic() {}
}
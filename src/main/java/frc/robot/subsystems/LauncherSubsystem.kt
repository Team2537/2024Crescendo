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
  
    val leftLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.LEFT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)
    val rightLauncherMotor : CANSparkFlex = CANSparkFlex(LauncherConstants.RIGHT_LAUNCHER_PORT, CANSparkLowLevel.MotorType.kBrushless)


    val leftPID = leftLauncherMotor.pidController

    init {
        val driveTab: ShuffleboardTab = Shuffleboard.getTab("Launcher Subsystem")

        AnglePID = angleMotor.pidController

        AnglePID.p = LauncherConstants.ANGLE_KP
        AnglePID.i = LauncherConstants.ANGLE_KI
        AnglePID.d = LauncherConstants.ANGLE_KD
      
        leftPID.p = LauncherConstants.LAUNCHER_P
        leftPID.i = LauncherConstants.LAUNCHER_I

        rightLauncherMotor.follow(leftLauncherMotor, true)


        tab.addNumber("Motor1 Velocity") {
            leftLauncherMotor.encoder.velocity
            rightLauncherMotor.encoder.velocity
        }

        driveTab.addNumber("Motor Velocity") {
            angleMotor.encoder.velocity
        }

    }

    fun setAngle(angle: Double) {
        AnglePID.setReference(angle, CANSparkBase.ControlType.kPosition)
    }
    
    fun setRawLaunchSpeed(rA: Double, rB: Double) {
        leftLauncherMotor.set(rA)
    }

    fun setLaunchVelocity(velocity: Double){
        leftPID.setReference(velocity, CANSparkBase.ControlType.kVelocity)
    }


    override fun periodic() {}
}
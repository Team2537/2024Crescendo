package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants


object LauncherSubsystem : SubsystemBase() {
    val leftLauncherMotor : CANSparkFlex = CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless)
    val rightLauncherMotor : CANSparkFlex = CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless)


    val leftPID = leftLauncherMotor.pidController

    init {
        val tab = Shuffleboard.getTab("Launcher Subsystem")

        leftPID.p = LauncherConstants.LAUNCHER_P
        leftPID.i = LauncherConstants.LAUNCHER_I

        rightLauncherMotor.follow(leftLauncherMotor, true)


        tab.addNumber("Motor1 Velocity") {
            leftLauncherMotor.encoder.velocity
            rightLauncherMotor.encoder.velocity
        }
    }
    fun setRawLaunchSpeed(rA: Double, rB: Double) {
        leftLauncherMotor.set(rA)
    }

    fun setLaunchVelocity(velocity: Double){
        leftPID.setReference(velocity, CANSparkBase.ControlType.kVelocity)
    }

    override fun periodic() {
        // This method will be called once per scheduler run

    }
}
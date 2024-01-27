package frc.robot.subsystems

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object LauncherSubsystem : SubsystemBase() {
    val LauncherMotor1 : CANSparkFlex = CANSparkFlex(1, CANSparkLowLevel.MotorType.kBrushless)
    val LauncherMotor2 : CANSparkFlex = CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless)

    private val kP : Double = 0.0002
    private val kI : Double = 0.00001
    private val kD : Double = 0.00001

    val pid1 = LauncherMotor1.pidController
    val pid2 = LauncherMotor2.pidController

    init {
        val driveTab = Shuffleboard.getTab("Launcher Subsystem")

        pid1.p = kP
        pid1.i = kI

        pid2.p = kP
        pid2.i = kI

        driveTab.addNumber("Motor1 Velocity") {
            LauncherMotor1.encoder.velocity
            LauncherMotor2.encoder.velocity
        }
    }
    fun set(rA: Double, rB: Double) {
        LauncherMotor1.set(rA)
        LauncherMotor2.set(rB)
    }

    fun setPosition(pos: Double) {
        pid1.setReference(pos, CANSparkBase.ControlType.kPosition)
        pid2.setReference(pos, CANSparkBase.ControlType.kPosition)
    }

    fun setLaunchVelocity(velocity: Double){
        pid1.setReference(velocity, CANSparkBase.ControlType.kVelocity)
        pid2.setReference(velocity, CANSparkBase.ControlType.kVelocity)
    }

    override fun periodic() {
        // This method will be called once per scheduler run

    }
}
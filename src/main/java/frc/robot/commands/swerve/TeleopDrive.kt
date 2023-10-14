// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.SwerveSubsystem
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow

class TeleopDrive(
    private val swerve: SwerveSubsystem,
    private val vX: DoubleSupplier,
    private val vY: DoubleSupplier,
    private val omega: DoubleSupplier,
    private val driveMode: BooleanSupplier,
    private val isOpenLoop: Boolean,
    private val headingCorrection: Boolean
) : CommandBase() {
    private val controller: SwerveController
    private val timer = Timer()
    private var angle = 0.0
    private var lastTime = 0.0


    init {
        controller = swerve.getSwerveController()
        if (headingCorrection) {
            timer.start()
        }
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    override fun initialize() {
        if (headingCorrection) {
            lastTime = timer.get()
        }
    }

    override fun execute() {
        val xVelocity: Double = vX.asDouble.pow(3.0)
        val yVelocity: Double = vY.asDouble.pow(3.0)
        val angVelocity: Double = omega.asDouble.pow(3.0)
        SmartDashboard.putNumber("vX", xVelocity)
        SmartDashboard.putNumber("vY", yVelocity)
        SmartDashboard.putNumber("omega", angVelocity)
        if (headingCorrection) {
            // Estimate the desired angle in radians.
            angle += angVelocity * (timer.get() - lastTime) * controller.config.maxAngularVelocity
            // Get the desired ChassisSpeeds given the desired angle and current angle.
            val correctedChassisSpeeds = controller.getTargetSpeeds(
                xVelocity, yVelocity, angle,
                swerve.getHeading().radians
            )
            // Drive using given data points.
            swerve.drive(
                SwerveController.getTranslation2d(correctedChassisSpeeds),
                correctedChassisSpeeds.omegaRadiansPerSecond,
                driveMode.asBoolean,
                isOpenLoop
            )
            lastTime = timer.get()
        } else {
            // Drive using raw values.
            swerve.drive(
                Translation2d(xVelocity * controller.config.maxSpeed, yVelocity * controller.config.maxSpeed),
                angVelocity * controller.config.maxAngularVelocity,
                driveMode.asBoolean, isOpenLoop
            )
        }
    }
}
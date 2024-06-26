// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * A command that controls the swerve drive using joystick inputs.
 * @param vForward The x velocity of the robot.
 * @param vStrafe The y velocity of the robot.
 * @param omega The angular velocity of the robot.
 * @param driveMode Boolean supplier that returns true if the robot should drive in field-oriented mode.
 * @param slowMode Boolean supplier that returns true if the robot should drive in slow mode.
 * @see SwerveSubsystem
 */
class TeleopDriveCommand(
    vForward: DoubleSupplier,
    vStrafe: DoubleSupplier,
    omega: DoubleSupplier,
    slowMode: DoubleSupplier
) : Command() {
    private val vForward: DoubleSupplier
    private val vStrafe: DoubleSupplier
    private val omega: DoubleSupplier
    private val slowMode: DoubleSupplier
    private val controller: SwerveController
    private val swerve: SwerveSubsystem

    init {
        this.swerve = SwerveSubsystem
        this.vForward = vForward
        this.vStrafe = vStrafe
        this.omega = omega
        this.slowMode = slowMode
        controller = swerve.getSwerveController()
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        var forwardVelocity = vForward.asDouble
        var strafeVelocity = vStrafe.asDouble
        var angVelocity = omega.asDouble
        var slowMode = slowMode.asDouble
        SmartDashboard.putNumber("vX", forwardVelocity)
        SmartDashboard.putNumber("vY", strafeVelocity)
        SmartDashboard.putNumber("omega", angVelocity)

        var slowModeMultiplier: Double = (-0.8 * slowMode) + 1.0

        forwardVelocity *= slowModeMultiplier
        strafeVelocity *= slowModeMultiplier
        angVelocity *= slowModeMultiplier


        // Drive using raw values.
        swerve.drive(
            Translation2d(forwardVelocity * swerve.maximumSpeed, strafeVelocity * swerve.maximumSpeed),
            angVelocity * controller.config.maxAngularVelocity,
            swerve.fieldOriented,
        )
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean {
        return false
    }
}

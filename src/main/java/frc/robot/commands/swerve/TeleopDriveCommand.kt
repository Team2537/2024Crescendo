// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swerve

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * A command that controls the swerve drive using joystick inputs.
 * @param vX The x velocity of the robot.
 * @param vY The y velocity of the robot.
 * @param omega The angular velocity of the robot.
 * @param driveMode Boolean supplier that returns true if the robot should drive in field-oriented mode.
 * @param slowMode Boolean supplier that returns true if the robot should drive in slow mode.
 * @see SwerveSubsystem
 */
class TeleopDriveCommand(
    vX: DoubleSupplier,
    vY: DoubleSupplier,
    omega: DoubleSupplier,
    driveMode: BooleanSupplier,
    slowMode: BooleanSupplier,
) : Command() {
    private val vX: DoubleSupplier
    private val vY: DoubleSupplier
    private val omega: DoubleSupplier
    private val driveMode: BooleanSupplier
    private val slowMode: BooleanSupplier
    private val controller: SwerveController
    private val swerve: SwerveSubsystem


    init {
        this.swerve = SwerveSubsystem
        this.vX = vX
        this.vY = vY
        this.omega = omega
        this.driveMode = driveMode
        this.slowMode = slowMode
        controller = swerve.getSwerveController()
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve)
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        var xVelocity = vX.asDouble
        var yVelocity = vY.asDouble
        var angVelocity = omega.asDouble
        val slowMode = slowMode.asBoolean
        SmartDashboard.putNumber("vX", xVelocity)
        SmartDashboard.putNumber("vY", yVelocity)
        SmartDashboard.putNumber("omega", angVelocity)

        if (slowMode) {
            xVelocity *= 0.6
            yVelocity *= 0.6
            angVelocity *= 0.6
        }

        // Drive using raw values.
        swerve.drive(
            Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
            angVelocity * controller.config.maxAngularVelocity,
            driveMode.asBoolean,
        )
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}

    /** @suppress */
    override fun isFinished(): Boolean {
        return false
    }
}

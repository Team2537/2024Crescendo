// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.swervedrive.drivebase

import SwerveSubsystem
import SwerveSubsystem.drive
import SwerveSubsystem.getSwerveController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * An example command that uses an example subsystem.
 */
class TeleopDrive(
    vX: DoubleSupplier, vY: DoubleSupplier, omega: DoubleSupplier,
    driveMode: BooleanSupplier, slowMode: BooleanSupplier
) : CommandBase() {
    private val vX: DoubleSupplier
    private val vY: DoubleSupplier
    private val omega: DoubleSupplier
    private val driveMode: BooleanSupplier
    private val slowMode: BooleanSupplier
    private val controller: SwerveController
    private val swerve: SwerveSubsystem

    /**
     * Creates a new ExampleCommand.
     *
     * @param swerve The subsystem used by this command.
     */
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

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        var xVelocity = vX.asDouble
        var yVelocity = vY.asDouble
        var angVelocity = omega.asDouble
        val slowMode = slowMode.asBoolean
        SmartDashboard.putNumber("vX", xVelocity)
        SmartDashboard.putNumber("vY", yVelocity)
        SmartDashboard.putNumber("omega", angVelocity)

        if(slowMode){
            xVelocity *= 0.6
            yVelocity *= 0.6
            angVelocity *= 0.6
        }

        // Drive using raw values.
        swerve.drive(
            Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
            angVelocity * controller.config.maxAngularVelocity,
            driveMode.asBoolean
        )
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
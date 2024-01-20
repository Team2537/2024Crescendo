package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.AutoLog

interface SwerveIO {
    @AutoLog
    class SwerveIOInputs{
        var states: Array<SwerveModuleState> = emptyArray()
        var heading: Rotation2d = Rotation2d(0.0, 0.0)
        var robotPose: Pose2d = Pose2d()
    }

    /** Updates the set of loggable inputs. */
    fun updateInputs(inputs: SwerveIOInputs)

    /** Drive method that translates and rotates the robot. */
    fun drive(translation: Translation2d, rotation: Double, fieldOriented: Boolean)

    /** Advanced drive method with a custom center of rotation. */
    fun drive(translation: Translation2d, rotation: Double, fieldOriented: Boolean, centerOfRotation: Translation2d)

    /** Simple drive method using ChassisSpeeds. */
    fun drive(velocity: ChassisSpeeds)

    /** Method to reset the odometry of the robot. */
    fun resetOdometry(initialHolonomicPose: Pose2d)

    /** Method to get the current pose of the robot. */
    fun getPose(): Pose2d

    /** Method to display a desired trajectory to a field2d object. */
    fun postTrajectory(trajectory: Trajectory){}

    /** Method to zero the gyro. */
    fun zeroGyro()

    /** Method to toggle the motor's brakes. */
    fun setMotorBrake(brake: Boolean)
}
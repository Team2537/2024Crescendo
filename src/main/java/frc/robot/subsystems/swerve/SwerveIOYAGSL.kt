package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import swervelib.SwerveDrive

class SwerveIOYAGSL(val swerve: SwerveDrive) : SwerveIO {

    override fun updateInputs(inputs: SwerveIO.SwerveIOInputs) {
        inputs.states = swerve.states
        inputs.heading = swerve.yaw
        inputs.robotPose = swerve.pose
    }

    override fun drive(translation: Translation2d, rotation: Double, fieldOriented: Boolean) {
        swerve.drive(translation, rotation, fieldOriented, false)
    }

    override fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
        centerOfRotation: Translation2d
    ) {
        swerve.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    override fun drive(velocity: ChassisSpeeds) {
        swerve.drive(velocity)
    }

    override fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerve.resetOdometry(initialHolonomicPose)
    }

    override fun getPose(): Pose2d {
        return swerve.pose
    }

    override fun zeroGyro() {
        swerve.zeroGyro()
    }

    override fun setMotorBrake(brake: Boolean) {
        swerve.setMotorIdleMode(brake)
    }


}
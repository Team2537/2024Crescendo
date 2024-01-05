package frc.robot.commands.swerve

import SwerveSubsystem
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants

class FollowTrajectory(trajectory: PathPlannerTrajectory, resetOdometry: Boolean) :
    SequentialCommandGroup() {
    val drivebase = SwerveSubsystem

    init {

        addRequirements(drivebase)
        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.initialHolonomicPose)
        }
        addCommands(
            PPSwerveControllerCommand(
                trajectory,
                { drivebase.getPose() },
                Constants.Auton.xAutoPID.createPIDController(),
                Constants.Auton.yAutoPID.createPIDController(),
                Constants.Auton.angleAutoPID.createPIDController(),
                { chassisSpeeds: ChassisSpeeds? -> drivebase.setChassisSpeeds(chassisSpeeds!!) },
                drivebase,
            ),
        )
    }
}

package frc.robot.commands.swerve

import frc.robot.subsystems.SwerveSubsystem
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
                Constants.Auto.xAutoPID.createPIDController(),
                Constants.Auto.yAutoPID.createPIDController(),
                Constants.Auto.angleAutoPID.createPIDController(),
                { chassisSpeeds: ChassisSpeeds? -> drivebase.setChassisSpeeds(chassisSpeeds!!) },
                drivebase,
            ),
        )
    }
}

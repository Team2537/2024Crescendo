package lib.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Constants
import frc.robot.subsystems.SwerveSubsystem

fun getPath(traj: ChoreoTrajectory, isRed: Boolean, drivebase: SwerveSubsystem, parallel: Command = InstantCommand()): Command {
    return Choreo.choreoSwerveCommand(
        traj,
        drivebase::pose,
        Constants.Auto.xAutoPID,
        Constants.Auto.yAutoPID,
        Constants.Auto.thetaAutoPID,
        drivebase::setChassisSpeeds,
        { isRed },
        drivebase
    ).alongWith(parallel)
}

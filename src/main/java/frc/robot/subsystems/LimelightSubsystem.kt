package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight
import lib.vision.LimelightHelpers
import lib.vision.VisionMeasurement

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {

    val odometryLimelight: Limelight = Limelight("limelight-odom") // FIXME
    val pieceLimelight: Limelight = Limelight("limelight-TODO") // FIXME
}

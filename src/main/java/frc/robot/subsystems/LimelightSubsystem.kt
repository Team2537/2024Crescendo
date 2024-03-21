package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {
    val odometryLimelight: Limelight = Limelight("limelight-odom")
    val intakeLimelight: Limelight = Limelight("limelight-intake")

    init {
        odometryLimelight.setTargetTag(7)
    }
}

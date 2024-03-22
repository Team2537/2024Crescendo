package frc.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight
import java.sql.Driver

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {
    val odometryLimelight: Limelight = Limelight("limelight-odom")
    val intakeLimelight: Limelight = Limelight("limelight-intake")

    init {
        odometryLimelight.setTargetTag(7)
    }

    override fun periodic() {
        if(DriverStation.getAlliance().isPresent){
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                odometryLimelight.setTargetTag(4)
            } else {
                odometryLimelight.setTargetTag(7)
            }
        }
    }
}

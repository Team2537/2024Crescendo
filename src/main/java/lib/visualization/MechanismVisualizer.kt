package lib.visualization

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class MechanismVisualizer(private val root: Supplier<Pose3d>, private val name: String) {

    data class Ligament(
        var parent: Optional<Ligament>,
        var child: Optional<Ligament>,
        val length: Measure<Distance>,
        val angleSupplier: DoubleSupplier
    )

    private val ligaments: MutableList<Ligament> = mutableListOf()

    fun addLigament(length: Measure<Distance>, angleSupplier: DoubleSupplier) {
        if (ligaments.size == 0) {
            ligaments.add(Ligament(Optional.empty(), Optional.empty(), length, angleSupplier))
        } else {
            val parent = ligaments[ligaments.size - 1]
            ligaments.add(Ligament(Optional.of(parent), Optional.empty(), length, angleSupplier))
            parent.child = Optional.of(ligaments[ligaments.size - 1])
        }
    }

    fun render(){
        val mechanism: MutableList<Pose3d> = mutableListOf()
        val rootPose = root.get()
        mechanism.add(rootPose)
        var parentPose = rootPose
        ligaments.forEachIndexed { index, ligament ->
            val childPose = parentPose.transformBy(
                Transform3d(
                    Translation3d(0.0, 0.0, ligament.length into Units.Meters),
                    Rotation3d(0.0, ligament.angleSupplier.asDouble, 0.0)
                )
            )
            mechanism.add(childPose)
            parentPose = childPose
        }
        Logger.recordOutput("mechanisms/$name", *mechanism.toTypedArray())
    }

}
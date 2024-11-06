package lib.math.poseestimation;

import edu.wpi.first.math.geometry.Twist2d;

public class WeightedTwist2d extends Twist2d {
    public final double fom;

    WeightedTwist2d(double dx, double dy, double dtheta, double fom) {
        super(dx, dy, dtheta);
        this.fom = fom;
    }

    @Override
    public String toString() {
        return "WeightedTwist2d{" +
                "fom=" + fom +
                ", dx=" + dx +
                ", dy=" + dy +
                ", dtheta=" + dtheta +
                '}';
    }
}

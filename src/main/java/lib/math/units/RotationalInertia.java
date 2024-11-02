package lib.math.units;

import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;

public class RotationalInertia extends Unit<RotationalInertia> {
    RotationalInertia(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(RotationalInertia.class, toBaseConverter, fromBaseConverter, name, symbol);
    }

    RotationalInertia(double baseUnitEquivalent, String name, String symbol) {
        super(RotationalInertia.class, baseUnitEquivalent, name, symbol);
    }
}

package lib.math.units;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mult;
import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;

public class Area extends Unit<Area> {
    Area(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(Area.class, toBaseConverter, fromBaseConverter, name, symbol);
    }

    Area(double baseUnitEquivalent, String name, String symbol) {
        super(Area.class, baseUnitEquivalent, name, symbol);
    }

    public static Area derive(Mult<? extends Distance, ? extends Distance> x) {
        Unit<? extends Distance> a = x.unitA();
        Unit<? extends Distance> b = x.unitB();

        UnaryFunction fromBase = a.getConverterFromBase().mult(b.getConverterFromBase());

        UnaryFunction toBase = a.getConverterToBase().mult(b.getConverterToBase());

        return new Area(
                toBase,
                fromBase,
                a.name() + " by " + b.name(),
                a.symbol() + b.symbol()
        );

    }

}

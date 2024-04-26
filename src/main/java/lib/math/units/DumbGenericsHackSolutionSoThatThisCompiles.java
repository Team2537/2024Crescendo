package lib.math.units;

import edu.wpi.first.units.Per;
import edu.wpi.first.units.Unit;

public final class DumbGenericsHackSolutionSoThatThisCompiles {
    private DumbGenericsHackSolutionSoThatThisCompiles(){}

    @SuppressWarnings("unchecked")
    public static <N extends Unit<N>, D extends Unit<D>> Per<?, ?> makePer(Class<N> nClass, Class<D> dClass, Unit<?> num, Unit<?> den){
        return Per.combine((N)num, (D)den);
    }
}

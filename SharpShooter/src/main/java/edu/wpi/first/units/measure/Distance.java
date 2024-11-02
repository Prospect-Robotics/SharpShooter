package edu.wpi.first.units.measure;

import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;

public class Distance extends Unit<Distance> {
    Distance(double baseUnitEquivalent, String name, String symbol) {
        super(Distance.class, baseUnitEquivalent, name, symbol);
    }

    Distance(
            UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(Distance.class, toBaseConverter, fromBaseConverter, name, symbol);
    }
}

package edu.wpi.first.units.measure;

import edu.wpi.first.units.UnaryFunction;
import edu.wpi.first.units.Unit;

public class Mass extends Unit<Mass> {
    /** Creates a new unit with the given name and multiplier to the base unit. */
    Mass(double baseUnitEquivalent, String name, String symbol) {
        super(Mass.class, baseUnitEquivalent, name, symbol);
    }

    Mass(UnaryFunction toBaseConverter, UnaryFunction fromBaseConverter, String name, String symbol) {
        super(Mass.class, toBaseConverter, fromBaseConverter, name, symbol);
    }
}

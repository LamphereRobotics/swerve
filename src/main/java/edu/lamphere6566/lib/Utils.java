package edu.lamphere6566.lib;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;

public final class Utils {
    public static DoubleSupplier withDeadband(DoubleSupplier input, double deadband) {
        return () -> MathUtil.applyDeadband(input.getAsDouble(), deadband);
    }

    public static DoubleSupplier inverted(DoubleSupplier input) {
        return () -> -input.getAsDouble();
    }
}

package org.firstinspires.ftc.teamcode.UTILITIES;

import org.gentrifiedApps.gentrifiedAppsUtil.classes.generics.pointClasses.Point;

public final class MathFunctions {

    private MathFunctions() {
        // Private constructor to prevent instantiation of utility class
    }

    public static boolean withinEpsilon(double value, double value2, double epsilon) {
        return (Math.abs(value2 - value) < epsilon);
    }

    // Overloaded method to support Kotlin's default parameter value (epsilon = 0.005)
    public static boolean withinEpsilon(double value, double value2) {
        return withinEpsilon(value, value2, 0.005);
    }

    /**
     * Returns the average of an array of values
     * @param values The values to get the average of
     * @return The average of the values
     */
    public static double averageOf(double[] values) {
        if (values.length == 0) return 0.0;
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }

    /**
     * Returns the quadrant of a point
     * @param pose The point to get the quadrant of
     * @return The quadrant of the point
     * 1 | 2
     * -----
     * 3 | 4
     */


    /**
     * Returns three-fourths of a number
     * @param amount The number to get three-fourths of
     * @return Three-fourths of the number
     */
    public static int threeFourths(int amount) {
        return amount / 4 * 3;
    }

    /**
     * Normalizes a delta to be between -180 and 180
     * @param delta The delta to normalize
     * @return The normalized delta
     */
    public static double normDelta(double delta) {
        return (delta % 180 + 180) % 180;
    }

    /**
     * Returns if a value is within a range
     * @param value The value to check
     * @param min The minimum value
     * @param max The maximum value
     */
    public static boolean inRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    /**
     * Returns if a value is within a tolerance of another value
     * @param value The value to check
     * @param value2 The value to check against
     * @param tolerance The tolerance
     * @return Boolean if the value is within the tolerance of the other value
     */
    public static boolean inTolerance(double value, double value2, double tolerance) {
        return value >= (value2 - tolerance) && value <= (value2 + tolerance);
    }

    public static boolean inTolerance(int value, int value2, int tolerance) {
        return inTolerance((double) value, (double) value2, (double) tolerance);
    }

    /**
     * Returns the angle between two points in degrees
     * @param P1 The first point
     * @param P2 The second point
     */


    /**
     * Returns the distance between two points
     //* @param P1 The first point
     * //@param P2 The second point
     */




    public static double getError(double set, double current) {
        return Math.abs(set - current);
    }

    public static double clip(double input, double min, double max) {
        if (input < min) {
            return min;
        } else if (input > max) {
            return max;
        } else {
            return input;
        }
    }

    public static double round(double input, int places) {
        if (places < 0) {
            throw new IllegalArgumentException("Decimal places must be non-negative.");
        }
        double factor = Math.pow(10.0, places);
        return Math.round(input * factor) / factor;
    }

    public static double ticksToInches(int ticks, double ticksPerIn) {
        return (double) ticks / ticksPerIn;
    }

    public static double lpnorm(double[] input) {
        double sum = 0.0;
        for (double it : input) {
            sum += (it * it);
        }
        return Math.sqrt(sum);
    }
}
package frc.bearbotics.math;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A utility class for performing quadratic curve interpolation based on a set of input-output
 * pairs.
 */
public class QuadraticCurveInterpolator {
  private final double c = 1;
  // the coefficient for c in any quadratic equation is one
  private Map<Double, Double> inputMap;
  private List<Double> inputKeys;

  /**
   * Constructs an QuadraticCurveInterpolator object with the given input-output map.
   *
   * @param inputMap The map representing the input-output pairs.
   */
  public QuadraticCurveInterpolator(Map<Double, Double> inputMap) {
    this.inputMap = inputMap;
    this.inputKeys = new ArrayList<>(inputMap.keySet());
  }

  /**
   * Calculates the output value for a given input using 3 point quadratic curve interpolation.
   *
   * @param input The input value.
   * @return The interpolated output value.
   */
  public double calculate(double input) {
    // Get the value in our map that is closest to our input
    double center = getNearestTarget(input);

    // Find the index of our value element
    int centerIndex = inputKeys.indexOf(center);

    // Find the keys to the left (idx - 1) and right (idx + 1) of our center key
    double x1 = centerIndex == 0 ? inputKeys.get(0) : inputKeys.get(centerIndex - 1);
    double x2 = center;
    double x3 =
        centerIndex == (inputKeys.size() - 1)
            ? inputKeys.get(centerIndex)
            : inputKeys.get(centerIndex + 1);

    // Find the respective values
    double y1 = inputMap.get(x1);
    double y2 = inputMap.get(x2);
    double y3 = inputMap.get(x3);

    // Interpolate a quadratic from the three points

    SystemOfThreeEquations coefficients =
        new SystemOfThreeEquations(
            Math.pow(x1, 2),
            Math.pow(x2, 2),
            Math.pow(x3, 2), // represents the x-squared in a quadratic
            x1,
            x2,
            x3, // represents the x in a quadratic
            c,
            c,
            c, // represents the coeffecient that multiplys c in a quadratic
            y1,
            y2,
            y3); // represents the output values of the quadratic for the given value x
    // constructs quadratic based off of now calculated a b and c and calculates output
    return new Quadratic(coefficients.getX(), coefficients.getY(), coefficients.getZ())
        .calculate(input);
  }

  /**
   * Finds the nearest target input value to the given target value.
   *
   * @param target The target input value.
   * @return The nearest input value from the input map.
   */
  private double getNearestTarget(double target) {
    return inputKeys.stream()
        .min(Comparator.comparingDouble(key -> Math.abs(target - key)))
        .orElse(0.0);
  }

  /**
   * builds a quadratic off of a b and c coefficients
   *
   * @param a a cof.
   * @param b b cof.
   * @param c c cof.
   */
  private class Quadratic {
    double a;
    double b;
    double c;

    public Quadratic(double a, double b, double c) {
      this.a = a;
      this.b = b;
      this.c = c;
    }

    /**
     * calculates the output of the quadratic.
     *
     * @param input The input value.
     * @return the output value.
     */
    private double calculate(double input) {
      return (a * Math.pow(input, 2)) + (b * input) + c;
    }
  }
}

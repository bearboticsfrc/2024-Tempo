package frc.robot.util;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A utility class for performing exponential curve interpolation based on a set of input-output
 * pairs.
 */
public class ExponentialCurveInterpolator {
  private final double MAX_DISTANCE = 5.26;

  private Map<Double, Double> inputMap;
  private List<Double> inputsList;

  /**
   * Constructs an ExponentialCurveInterpolator object with the given input-output map.
   *
   * @param inputMap The map representing the input-output pairs.
   */
  public ExponentialCurveInterpolator(Map<Double, Double> inputMap) {
    this.inputMap = inputMap;
    this.inputsList = new ArrayList<>(inputMap.keySet());
  }

  /**
   * Calculates the output value for a given input using exponential curve interpolation.
   *
   * @param input The input value.
   * @return The interpolated output value.
   */
  public double calculate(double input) {
    double center = getNearestTarget(input);
    int centerIndex = inputsList.indexOf(center);

    double x2 = center;
    double x1 = centerIndex > 0 ? inputsList.get(centerIndex - 1) : 0;
    double x3 =
        centerIndex < inputsList.size() - 1 ? inputsList.get(centerIndex + 1) : MAX_DISTANCE;

    double y1 = (centerIndex > 0) ? inputMap.get(x1) : 0;
    double y2 = inputMap.get(x2);
    double y3 = inputMap.get(x3);

    return new QuadraticInterpolator(x1, x2, x3, y1, y2, y3).getValue(input);
  }

  /**
   * Finds the nearest target input value to the given target value.
   *
   * @param target The target input value.
   * @return The nearest input value from the input map.
   */
  private double getNearestTarget(double target) {
    return inputsList.stream()
        .min(Comparator.comparingDouble(key -> Math.abs(target - key)))
        .orElse(0.0);
  }

  /** A helper class for performing quadratic interpolation. */
  private class QuadraticInterpolator {
    private double a;
    private double b;
    private double c;

    /**
     * Constructs a QuadraticInterpolator object with the given input-output pairs.
     *
     * @param x1 The x-coordinate of the first point.
     * @param x2 The x-coordinate of the second point.
     * @param x3 The x-coordinate of the third point.
     * @param y1 The y-coordinate of the first point.
     * @param y2 The y-coordinate of the second point.
     * @param y3 The y-coordinate of the third point.
     */
    private QuadraticInterpolator(
        double x1, double x2, double x3, double y1, double y2, double y3) {
      double[][] d = {
        {Math.pow(x1, 2), x1, 1},
        {Math.pow(x2, 2), x2, 1},
        {Math.pow(x3, 2), x3, 1}
      };

      double[][] dA = {
        {y1, x1, 1},
        {y2, x2, 1},
        {y3, x3, 1}
      };

      double[][] dB = {
        {Math.pow(x1, 2), y1, 1},
        {Math.pow(x2, 2), y2, 1},
        {Math.pow(x3, 2), y3, 1}
      };

      double[][] dC = {
        {Math.pow(x1, 2), x1, y1},
        {Math.pow(x2, 2), x2, y2},
        {Math.pow(x3, 2), x3, y3}
      };

      a = getDeterminant(dA) / getDeterminant(d);
      b = getDeterminant(dB) / getDeterminant(d);
      c = getDeterminant(dC) / getDeterminant(d);
    }

    /**
     * Calculates the interpolated output value for a given input value.
     *
     * @param x The input value.
     * @return The interpolated output value.
     */
    private double getValue(double x) {
      return (a * Math.pow(x, 2)) + (b * x) + c;
    }

    /**
     * Calculates the determinant of a 3x3 matrix.
     *
     * @param matrix The 3x3 matrix.
     * @return The determinant value.
     */
    private double getDeterminant(double[][] matrix) {
      double x = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]));
      double y = (matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]));
      double z = (matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]));

      return x - y + z;
    }
  }
}

package frc.bearbotics.math;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A utility class for performing exponential curve interpolation based on a set of input-output
 * pairs.
 */
public class ExponentialCurveInterpolator {
  private Map<Double, Double> inputMap;
  private List<Double> inputKeys;

  /**
   * Constructs an ExponentialCurveInterpolator object with the given input-output map.
   *
   * @param inputMap The map representing the input-output pairs.
   */
  public ExponentialCurveInterpolator(Map<Double, Double> inputMap) {
    this.inputMap = inputMap;
    this.inputKeys = new ArrayList<>(inputMap.keySet());
  }

  /**
   * Calculates the output value for a given input using exponential curve interpolation.
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

    // Interpolate from the three points
    return new QuadraticInterpolator(
            new Translation2d(x1, y1), new Translation2d(x2, y2), new Translation2d(x3, y3))
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
}

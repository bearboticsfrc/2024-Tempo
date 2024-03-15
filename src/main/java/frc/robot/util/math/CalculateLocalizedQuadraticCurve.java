package frc.robot.util.math;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

public class CalculateLocalizedQuadraticCurve {
  Map<Double, Double> inputMap;
  List<Double> inputsList;
  double maxValue;

  /**
   * Constructs a new instance of a localized approximation protocol
   * CalculateLocalizedQuadraticCurve based off of its input it draws a quadratic equation around
   * the nearest three points including the center to the input
   *
   * @param inputMap the table of values to approximate the function off of.
   */
  public CalculateLocalizedQuadraticCurve(Map<Double, Double> inputMap) {
    this.inputMap = inputMap;
    this.inputsList = new ArrayList<Double>(inputMap.keySet());
    Collections.sort(inputsList);
    this.maxValue = inputsList.get(inputsList.size() - 2);
  }

  public double calculate(double input) {
    double center = getNearestTarget(input);
    int centerIndex = inputsList.indexOf(center);

    double x2 = center;
    double x1 = centerIndex > 0 ? inputsList.get(centerIndex - 1) : 0;
    double x3 = centerIndex < inputsList.size() - 1 ? inputsList.get(centerIndex + 1) : maxValue;

    double y1 = (centerIndex > 0) ? inputMap.get(x1) : 0;
    double y2 = inputMap.get(x2);
    double y3 = inputMap.get(x3);

    SystemOfThreeEquations equationCoefficients =
        new SystemOfThreeEquations(
            Math.pow(x1, 2), Math.pow(x2, 2), Math.pow(x3, 2), x1, x2, x3, 1, 1, 1, y1, y2, y3);

    QuadraticCurve localizQuadraticCurve =
        new QuadraticCurve(
            equationCoefficients.getX(), equationCoefficients.getY(), equationCoefficients.getZ());

    return localizQuadraticCurve.getValue(input);
  }

  private double getNearestTarget(double target) {
    return inputsList.stream()
        .min(Comparator.comparingDouble(key -> Math.abs(target - key)))
        .orElse(0.0);
  }
}

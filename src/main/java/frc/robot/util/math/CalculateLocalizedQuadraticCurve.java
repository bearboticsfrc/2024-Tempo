package frc.robot.util.math;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class CalculateLocalizedQuadraticCurve {
  Map<Double, Double> inputToOutput;
  List<Double> inputsList;
  double maxValue;

  /**
   * Constructs a new instance of a localized approximation protocol
   * CalculateLocalizedQuadraticCurve based off of its input it draws a quadratic equation around
   * the nearest three points including the center to the input
   *
   * @param table the table of values to approximate the function off of.
   */
  public CalculateLocalizedQuadraticCurve(Map<Double, Double> table) {
    inputToOutput = table;
    inputsList = new ArrayList<Double>(table.keySet());
    Collections.sort(inputsList);
    maxValue = inputsList.get(inputsList.size() - 2);
  }

  public double calculate(double input) {
    input = Math.min(input, maxValue);
    double x1;
    double x2;
    double x3;
    double y1;
    double y2;
    double y3;
    double center = nearestKey(input);
    System.out.println("center = " + center);
    x2 = center;
    if (inputsList.get(inputsList.indexOf(center)) == 0) {
      x1 = 0;
      y1 = 0;

      x3 = inputsList.get(inputsList.indexOf(center) + 1);

    } else {
      x1 = inputsList.get(inputsList.indexOf(center) - 1);
      y1 = inputToOutput.get(x1);
      x3 = inputsList.get(inputsList.indexOf(center) + 1);
    }

    y2 = inputToOutput.get(x2);
    y3 = inputToOutput.get(x3);
    SystemOfThreeEquations equationCoefficients =
        new SystemOfThreeEquations(
            Math.pow(x1, 2), Math.pow(x2, 2), Math.pow(x3, 2), x1, x2, x3, 1, 1, 1, y1, y2, y3);

    QuadraticCurve localizQuadraticCurve =
        new QuadraticCurve(
            equationCoefficients.getX(), equationCoefficients.getY(), equationCoefficients.getZ());

    return localizQuadraticCurve.getValue(input);
  }

  private double nearestKey(double target) {
    double minDiff = maxValue;
    double nearest = 0;
    for (double key : inputsList) {
      double diff = Math.abs((double) target - (double) key);
      if (diff < minDiff) {
        nearest = key;
        minDiff = diff;
      }
    }
    return nearest;
  }
}

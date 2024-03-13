package frc.robot.util;

import frc.robot.constants.manipulator.ShooterConstants;
import java.util.HashMap;
import java.util.List;

public class CalculateAnExponentialCurve {
  HashMap<Double, Double> inputToOutput;
  List<Double> inputsList;

  public CalculateAnExponentialCurve(HashMap<Double, Double> table, List<Double> inputs) {
    inputToOutput = table;
    inputsList = inputs;
  }

  public double calculate(double input) {
    double center = nearestKey(inputToOutput, input);
    double x1 = inputsList.get(inputsList.indexOf(center) - 1);
    double x2 = center;
    double x3 = inputsList.get(inputsList.indexOf(center) - 1);
    double y1 = inputToOutput.get(x1);
    double y2 = inputToOutput.get(x2);
    double y3 = inputToOutput.get(x3);
    Equation equation = new Equation(x1, x2, x3, y1, y2, y3);
    return equation.getValue(input);
  }

  public double nearestKey(HashMap<Double, Double> map, double target) {
    double minDiff = ShooterConstants.MAX_DISTANCE;
    double nearest = 0;
    for (double key : map.keySet()) {
      double diff = Math.abs((double) target - (double) key);
      if (diff < minDiff) {
        nearest = key;
        minDiff = diff;
      }
    }
    return nearest;
  }

  private class Equation {
    double a;
    double b;
    double c;
    double x;
    double y;
    double z;

    private Equation(double x1, double x2, double x3, double y1, double y2, double y3) {
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

    private double getValue(double x) {
      return (a * Math.pow(x, 2)) + (b * x) + c;
    }

    private double getDeterminant(double[][] matrix) {

      x = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]));

      y = (matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]));

      z = (matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]));
      return x - y + z;
    }
  }
}

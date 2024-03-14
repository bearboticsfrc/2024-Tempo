package frc.robot.util;

import frc.robot.constants.manipulator.ShooterConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class CalculateAnExponentialCurve {
  Map<Double, Double> inputToOutput;
  List<Double> inputsList;

  public CalculateAnExponentialCurve(Map<Double, Double> table) {
    inputToOutput = table;
    inputsList = new ArrayList<Double>(table.keySet());
    Collections.sort(inputsList);
  }

  public double calculate(double input) {
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

    } else if (inputsList.get(inputsList.indexOf(center)) == (inputsList.size() - 1)) {
      x1 = inputsList.get(inputsList.indexOf(center) - 1);
      y1 = inputToOutput.get(x1);
      x3 = ShooterConstants.MAX_DISTANCE;

    } else {
      x1 = inputsList.get(inputsList.indexOf(center) - 1);
      y1 = inputToOutput.get(x1);
      x3 = inputsList.get(inputsList.indexOf(center) + 1);
    }

    y2 = inputToOutput.get(x2);
    y3 = inputToOutput.get(x3);
    Equation equation = new Equation(x1, x2, x3, y1, y2, y3);
    return equation.getValue(input);
  }

  public double nearestKey(double target) {
    double minDiff = ShooterConstants.MAX_DISTANCE;
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

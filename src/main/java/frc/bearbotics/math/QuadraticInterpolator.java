package frc.bearbotics.math;

import edu.wpi.first.math.geometry.Translation2d;

/** A helper class for performing quadratic interpolation. */
public class QuadraticInterpolator {
  private final Translation2d point1;
  private final Translation2d point2;
  private final Translation2d point3;

  /**
   * Constructs a QuadraticInterpolator object with three input points.
   *
   * @param point1 The first point.
   * @param point2 The second point.
   * @param point3 The third point.
   */
  public QuadraticInterpolator(Translation2d point1, Translation2d point2, Translation2d point3) {
    this.point1 = point1;
    this.point2 = point2;
    this.point3 = point3;
  }

  /**
   * Calculates the interpolated output value for a given input value using quadratic interpolation.
   *
   * @param x The input value.
   * @return The interpolated output value.
   */
  public double calculate(double x) {
    double x1 = point1.getX();
    double x2 = point2.getX();
    double x3 = point3.getX();

    double[][] d = {
      {Math.pow(x1, 2), x1, 1},
      {Math.pow(x2, 2), x2, 1},
      {Math.pow(x3, 2), x3, 1}
    };

    double detD = getDeterminant(d);
    double coefficientA = getA(detD);
    double coefficientB = getB(detD);
    double coefficientC = getC(detD);

    return (coefficientA * Math.pow(x, 2)) + (coefficientB * x) + coefficientC;
  }

  /**
   * Calculates the 'A' coefficient for quadratic interpolation.
   *
   * @param detD The determinant of matrix d.
   * @return The 'A' coefficient.
   */
  private double getA(double detD) {
    double[][] dA = {
      {point1.getY(), point1.getX(), 1},
      {point2.getY(), point2.getX(), 1},
      {point3.getY(), point3.getX(), 1}
    };

    return getDeterminant(dA) / detD;
  }

  /**
   * Calculates the 'B' coefficient for quadratic interpolation.
   *
   * @param detD The determinant of matrix d.
   * @return The 'B' coefficient.
   */
  private double getB(double detD) {
    double[][] dB = {
      {Math.pow(point1.getX(), 2), point1.getY(), 1},
      {Math.pow(point2.getX(), 2), point2.getY(), 1},
      {Math.pow(point3.getX(), 2), point3.getY(), 1}
    };

    return getDeterminant(dB) / detD;
  }

  /**
   * Calculates the 'C' coefficient for quadratic interpolation.
   *
   * @param detD The determinant of matrix d.
   * @return The 'C' coefficient.
   */
  private double getC(double detD) {
    double x1 = point1.getX();
    double x2 = point2.getX();
    double x3 = point3.getX();

    double[][] dC = {
      {Math.pow(x1, 2), x1, point1.getY()},
      {Math.pow(x2, 2), x2, point2.getY()},
      {Math.pow(x3, 2), x3, point3.getY()}
    };

    return getDeterminant(dC) / detD;
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

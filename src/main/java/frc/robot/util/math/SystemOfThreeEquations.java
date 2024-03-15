package frc.robot.util.math;

/**
 * Represents a system of three linear equations in three variables (x, y, z) and provides methods
 * to solve the system and calculate values.
 */
public class SystemOfThreeEquations {

  /** The solution for variable x. */
  private double solvedX;

  /** The solution for variable y. */
  private double solvedY;

  /** The solution for variable z. */
  private double solvedZ;

  /**
   * Constructs a new instance of SystemOfThreeEquations with coefficients and constants of the
   * equations.
   *
   * @param cofX1 The coefficient of x in the first equation.
   * @param cofX2 The coefficient of x in the second equation.
   * @param cofX3 The coefficient of x in the third equation.
   * @param cofY1 The coefficient of y in the first equation.
   * @param cofY2 The coefficient of y in the second equation.
   * @param cofY3 The coefficient of y in the third equation.
   * @param cofZ1 The coefficient of z in the first equation.
   * @param cofZ2 The coefficient of z in the second equation.
   * @param cofZ3 The coefficient of z in the third equation.
   * @param d1 The constant term in the first equation.
   * @param d2 The constant term in the second equation.
   * @param d3 The constant term in the third equation.
   */
  public SystemOfThreeEquations(
      double cofX1,
      double cofX2,
      double cofX3,
      double cofY1,
      double cofY2,
      double cofY3,
      double cofZ1,
      double cofZ2,
      double cofZ3,
      double d1,
      double d2,
      double d3) {

    double[][] d = {
      {cofX1, cofY1, cofZ1},
      {cofX2, cofY2, cofZ2},
      {cofX3, cofY3, cofZ3}
    };

    double[][] dX = {
      {d1, cofY1, cofZ1},
      {d2, cofY2, cofZ2},
      {d3, cofY3, cofZ3}
    };

    double[][] dY = {
      {cofX1, d1, cofZ1},
      {cofX2, d2, cofZ2},
      {cofX3, d3, cofZ3}
    };

    double[][] dZ = {
      {cofX1, cofY1, d1},
      {cofX2, cofY2, d2},
      {cofX3, cofY3, d3}
    };

    solvedX = getDeterminant(dX) / getDeterminant(d);
    solvedY = getDeterminant(dY) / getDeterminant(d);
    solvedZ = getDeterminant(dZ) / getDeterminant(d);
  }

  /**
   * Calculates the determinant of a 3x3 matrix.
   *
   * @param matrix The 3x3 matrix for which the determinant is to be calculated.
   * @return The determinant of the matrix.
   */
  private double getDeterminant(double[][] matrix) {

    double x = (matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]));

    double y = (matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]));

    double z = (matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]));

    return x - y + z;
  }

  /**
   * @return the coefficent of x for the system.
   */
  public double getX() {
    return solvedX;
  }

  /**
   * @return the coefficent of y for the system.
   */
  public double getY() {
    return solvedY;
  }
  /**
   * @return the coefficent of z for the system.
   */
  public double getZ() {
    return solvedZ;
  }
}

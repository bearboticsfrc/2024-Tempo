package frc.bearbotics.math;

import Jama.Matrix;

/**
 * Represents a system of three linear equations with three variables (x, y, z) of the form: a1*x +
 * b1*y + c1*z = d1 a2*x + b2*y + c2*z = d2 a3*x + b3*y + c3*z = d3 Provides methods to solve the
 * system and obtain the values of x, y, and z.
 */
public class SystemOfThreeEquations {
  private Matrix coefficients;
  private Matrix constants;

  /**
   * Constructs a new instance of SystemOfThreeEquations with the given coefficients and constants.
   *
   * @param coefficientsArray Array representing the coefficients of the variables x, y, and z for
   *     each equation. Each row of the array corresponds to an equation, and each column represents
   *     a variable.
   * @param constantsArray Array representing the constants on the right-hand side of each equation.
   *     Each row of the array corresponds to an equation.
   */
  public SystemOfThreeEquations(double[] x, double[] y, double[] z, double[] d) {
    double[][] coefficientsArray = {
      {x[0], y[0], z[0]},
      {x[1], y[1], z[1]},
      {x[2], y[2], z[2]}
    };

    double[] constantsArray = {d[0], d[1], d[2]};

    this.coefficients = new Matrix(coefficientsArray);
    this.constants = new Matrix(constantsArray, 3);
  }

  /**
   * Solves the system of equations and returns the solution.
   *
   * @return An instance of SolvedSystem containing the values of x, y, and z that satisfy the
   *     system of equations.
   * @throws IllegalArgumentException if the system of equations has no unique solution (i.e., if
   *     the determinant is zero).
   */
  public SolvedSystem solve() {
    double determinant = coefficients.det();

    if (determinant == 0) {
      throw new IllegalArgumentException("The system of equations has no unique solution.");
    }

    return new SolvedSystem(coefficients.solve(constants));
  }
}

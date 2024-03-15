package frc.robot.util.math;

public class QuadraticCurve {
  double a;
  double b;
  double c;

  public QuadraticCurve(double cofA, double cofB, double cofC) {
    a = cofA;
    b = cofB;
    c = cofC;
  }

  public double getValue(double input) {
    return (a * Math.pow(input, 2)) + (b * input) + c;
  }
}

package frc.bearbotics.pid;

/** PID constants used to create PID controllers */
public class PIDConstants {
  /** P */
  public final double P;
  /** I */
  public final double I;
  /** D */
  public final double D;
  /** Integral range */
  public final double iZone;

  /**
   * Create a new PIDConstants object
   *
   * @param kP P
   * @param kI I
   * @param kD D
   * @param iZone Integral range
   */
  public PIDConstants(double P, double I, double D, double iZone) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.iZone = iZone;
  }

  /**
   * Create a new PIDConstants object
   *
   * @param P P
   * @param I I
   * @param D D
   */
  public PIDConstants(double P, double I, double D) {
    this(P, I, D, 0);
  }

  /**
   * Create a new PIDConstants object
   *
   * @param P P
   * @param D D
   */
  public PIDConstants(double P, double D) {
    this(P, 0, D);
  }

  /**
   * Create a new PIDConstants object
   *
   * @param P P
   */
  public PIDConstants(double P) {
    this(P, 0, 0);
  }
}

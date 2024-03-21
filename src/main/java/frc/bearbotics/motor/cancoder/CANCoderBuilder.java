package frc.bearbotics.motor.cancoder;

import edu.wpi.first.math.geometry.Rotation2d;

/** Builder class for constructing instances of the {@link CANCoders} class. */
public class CANCoderBuilder {
  private int id;
  private Rotation2d offsetDegrees;

  /**
   * Gets the ID of the CANCoder.
   *
   * @return The ID of the CANCoder.
   */
  public int getId() {
    return id;
  }

  /**
   * Sets the ID of the CANCoder.
   *
   * @param id The ID to set.
   * @return The current instance of the builder for method chaining.
   */
  public CANCoderBuilder setId(int id) {
    this.id = id;
    return this;
  }

  /**
   * Gets the offset in degrees for the CANCoder.
   *
   * @return The offset in degrees for the CANCoder.
   */
  public Rotation2d getOffsetDegrees() {
    return offsetDegrees;
  }

  /**
   * Sets the offset in degrees for the CANCoder.
   *
   * @param offsetDegrees The offset in degrees to set.
   * @return The current instance of the builder for method chaining.
   */
  public CANCoderBuilder setOffsetDegrees(Rotation2d offsetDegrees) {
    this.offsetDegrees = offsetDegrees;
    return this;
  }
}

package frc.robot.lib.SmartMotorController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Handles a motor controller and its configuration.
 */
public class SmartMotorController {
  /**
   * The master speed multiplier of the motor.
   */
  private final double m_multiplier;

  /**
   * The rate at which the motor should get faster.
   */
  private final double m_accelUp;

  /**
   * The rate at which the motor should get slower or change direction.
   */
  private final double m_accelDown;

  /**
   * The motor controller.
   */
  private final MotorController m_controller;

  /**
   * Initialize the motor and its configuration.
   *
   * @param invert The direction in which the motor should spin.
   * @param accelUp The rate at which the motor should get faster. 
   * @param accelDown The rate at which the motor should get slower or change direction.
   * @param multiplier The master speed multiplier of the motor.
   * @param controller The motor controller.
   */
  public SmartMotorController(boolean invert, double multiplier, double accelUp, double accelDown, MotorController controller) {
    m_multiplier = multiplier;
    m_accelUp = accelUp;
    m_accelDown = accelDown;
    m_controller = controller;
    m_controller.setInverted(invert);
  }

  /**
   * Initialize the motor and its configuration.
   * Assume that the acceleration rate is the same for both directions.
   *
   * @param invert The direction in which the motor should spin.
   * @param accel The rate at which the motor should change speed.
   * @param multiplier The master speed multiplier of the motor.
   * @param controller The motor controller.
   */
  public SmartMotorController(boolean invert, double multiplier, double accel, MotorController controller) {
    this(invert, multiplier, accel, accel, controller);
  }

  /**
   * Accelerate the motor towards the given velocity.
   *
   * @param velocity The requested velocity of the motor.
   * @return The actual velocity of the motor.
   */
  public double accelTo(double velocity) {
    double now = m_controller.get() / m_multiplier;

    double accelDir = Math.signum(velocity - now);
    double accelMag = (
      Math.signum(now) == Math.signum(velocity) &&
      Math.abs(velocity) < Math.abs(now) ?
      m_accelDown : m_accelUp
    );

    now += accelDir * accelMag;

    if (accelDir < 0) {
      now = Math.max(now, velocity);
    }
    else {
      now = Math.min(now, velocity);
    }

    m_controller.set(now * m_multiplier);
    return now;
  }

  /**
   * Set the motor immediately to a given velocity.
   *
   * @param velocity The requested velocity of the motor.
   */
  public void set(double velocity) {
    m_controller.set(velocity * m_multiplier);
  }

  /**
   * Toggle the motor between on and off.
   */
  public void toggle() {
    m_controller.set(m_controller.get() == 0.0 ? 1.0 : 0.0);
  }

  public MotorController getController() {
    return m_controller;
  }
}

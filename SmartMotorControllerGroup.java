package frc.robot.lib.SmartMotorController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Handles one side of a tank drive drive train.
 */
public class SmartMotorControllerGroup<T extends MotorController> {
  /**
   * The master motor controller which is followed by the others.
   */
  private final SmartMotorController m_controller;

  /**
   * Initialize the motors and their configuration.
   *
   * @param invert The direction in which the motors should spin.
   * @param accelUp The rate at which the motors should get faster. 
   * @param accelDown The rate at which the motors should get slower or change direction.
   * @param multiplier The master speed multiplier of the motors.
   * @param follow The function to follow the master motor controller.
   * @param controllers The motor controllers.
   */
  public SmartMotorControllerGroup(
    boolean invert, double multiplier, double accelUp, double accelDown,
    SmartMotorControllerFollower<T> follow, T... controllers
  ) {
    T master = controllers[0];
    m_controller = new SmartMotorController(invert, multiplier, accelUp, accelDown, master);

    for (T controller : controllers) {
      if (controller != master) {
        controller.setInverted(invert);

        follow.follow(master, controller);
      }
    }
  }

  /**
   * Initialize the motors and their configuration.
   * Assume that the acceleration rate is the same for both directions.
   *
   * @param invert The direction in which the motors should spin.
   * @param accel The rate at which the motors should change speed.
   * @param multiplier The master speed multiplier of the motors.
   * @param follow The function to follow the master motor controller.
   * @param controller The motor controllers.
   */
  public SmartMotorControllerGroup(
    boolean invert, double multiplier, double accel,
    SmartMotorControllerFollower<T> follow, T... controllers
  ) {
    this(invert, multiplier, accel, accel, follow, controllers);
  }

  /**
   * Accelerate the motors towards the given velocity.
   *
   * @param velocity The requested velocity of the motors.
   * @return The actual velocity of the motors.
   */
  public double accelTo(double velocity) {
    return m_controller.accelTo(velocity);
  }

  /**
   * Set the motors immediately to a given velocity.
   *
   * @param velocity The requested velocity of the motors.
   */
  public void forceTo(double velocity) {
    m_controller.set(velocity);
  }
}

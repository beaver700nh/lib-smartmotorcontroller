package frc.robot.lib.SmartMotorController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Handles one side of a tank drive drive train.
 */
public class SmartMotorControllerGroup<T extends MotorController> extends SmartMotorController {
  T[] m_controllers;

  /**
   * Initialize the motors and their configuration.
   *
   * @param invert The direction in which the motors should spin.
   * @param multiplier The master speed multiplier of the motors.
   * @param accelUp The rate at which the motors should get faster. 
   * @param accelDown The rate at which the motors should get slower or change direction.
   * @param follow The function to follow the master motor controller.
   * @param controllers The motor controllers.
   */
  @SuppressWarnings("unchecked")
  public SmartMotorControllerGroup(
    boolean invert, double multiplier, double accelUp, double accelDown,
    SmartMotorControllerFollower<T> follow, T... controllers
  ) {
    super(invert, multiplier, accelUp, accelDown, controllers[0]);

    m_controllers = controllers;

    for (T controller : controllers) {
      if (controller != m_controller) {
        controller.setInverted(invert);

        follow.follow(controllers[0], controller);
      }
    }
  }

  /**
   * Initialize the motors and their configuration.
   * Assume that the acceleration rate is the same for both directions.
   *
   * @param invert The direction in which the motors should spin.
   * @param multiplier The master speed multiplier of the motors.
   * @param accel The rate at which the motors should change speed.
   * @param follow The function to follow the master motor controller.
   * @param controllers The motor controllers.
   */
  @SuppressWarnings("unchecked")
  public SmartMotorControllerGroup(
    boolean invert, double multiplier, double accel,
    SmartMotorControllerFollower<T> follow, T... controllers
  ) {
    this(invert, multiplier, accel, accel, follow, controllers);
  }

  /**
   * Initialize the motors and their configuration.
   * Assume that the acceleration rate is unused and set it to zero.
   *
   * @param invert The direction in which the motors should spin.
   * @param multiplier The master speed multiplier of the motors.
   * @param follow The function to follow the master motor controller.
   * @param controllers The motor controllers.
   */
  @SuppressWarnings("unchecked")
  public SmartMotorControllerGroup(
    boolean invert, double multiplier,
    SmartMotorControllerFollower<T> follow, T... controllers
  ) {
    this(invert, multiplier, 0.0, follow, controllers);
  }

  public MotorController[] getControllers() {
    return m_controllers;
  }
}

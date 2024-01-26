package frc.robot.lib.SmartMotorController;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix.motorcontrol.IMotorController;

import com.revrobotics.CANSparkBase;

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
   * @param controllers The motor controllers.
   */
  public SmartMotorControllerGroup(boolean invert, double multiplier, double accelUp, double accelDown, T... controllers) {
    T master = controllers[0];
    m_controller = new SmartMotorController(invert, multiplier, accelUp, accelDown, master);

    for (T controller : controllers) {
      if (controller != master) {
        controller.setInverted(invert);

        if (controller instanceof IMotorController)
          ((IMotorController) controller).follow((IMotorController) master);
        else if (controller instanceof CANSparkBase)
          ((CANSparkBase) controller).follow((CANSparkBase) master);
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
   * @param controller The motor controllers.
   */
  public SmartMotorControllerGroup(boolean invert, double multiplier, double accel, T... controllers) {
    this(invert, multiplier, accel, accel, controllers);
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

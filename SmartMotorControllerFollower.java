// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.SmartMotorController;

/** Defines a function for a motor controller to follow another. */
public interface SmartMotorControllerFollower<T> {
  void follow(T master, T follower);
}

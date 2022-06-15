// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends CommandBase {

  private final WPI_TalonSRX left = new WPI_TalonSRX(LiftConstants.kLeftMotorPort);
  private final WPI_TalonSRX right = new WPI_TalonSRX(LiftConstants.kRightMotorPort);

  public Lift() {

  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {

  }

  public void liftRun(double speed){
    left.set(speed);
    right.follow(left);
  }

  public void leftStop(){
    left.set(0);
    right.set(0);
  }
 
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}

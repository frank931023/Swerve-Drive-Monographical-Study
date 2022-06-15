// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends CommandBase {

  private final WPI_TalonSRX trans = new WPI_TalonSRX(TransportConstants.kTransMotorPort);

  public TransportSubsystem() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  public void TransportRun(){
    trans.set(TransportConstants.kTransSpeed);
  }

  public void TransportStop(){
    trans.set(0);
  }

  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}

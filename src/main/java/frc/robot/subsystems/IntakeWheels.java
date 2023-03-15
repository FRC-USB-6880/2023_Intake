// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeWheelsConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWheels extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  /** Creates a new IntakeWheels. */
  public IntakeWheels() {
    m_motor = new CANSparkMax(CAN_IDs.intakeWheels, MotorType.kBrushless);
    m_motor.setInverted(IntakeWheelsConstants.kWheelsInverted);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(IntakeWheelsConstants.kCurrentLimit);
    m_encoder = m_motor.getEncoder();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase grabCone() {
    return this.runOnce(() -> m_motor.set(IntakeWheelsConstants.kConeGrabSpeed))
              .until(() -> m_motor.getOutputCurrent() > IntakeWheelsConstants.kCurrentWhenConeIsGrabbed)
              .withTimeout(1)
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("GrabeCone");
  }

  public CommandBase releaseCone() {
    return this.runOnce(() -> m_motor.set(IntakeWheelsConstants.kConeReleaseSpeed))
              .withTimeout(0.5)
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("ReleaseCone");
  }

  public CommandBase grabCube() {
    return this.runOnce(() -> m_motor.set(IntakeWheelsConstants.kCubeGrabSpeed))
              .until(() -> m_motor.getOutputCurrent() > IntakeWheelsConstants.kCurrentWhenCubeIsGrabbed)
              .withTimeout(1)
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("grabCube");
  }

  public CommandBase releaseCube() {
    return this.runOnce(() -> m_motor.set(IntakeWheelsConstants.kCubeReleaseSpeed))
              .withTimeout(0.5)
              .finallyDo((interrupted) -> m_motor.set(0))
              .withName("ReleaseCube");
  }

}

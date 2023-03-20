// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArm extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;

  /** Creates a new ExampleSubsystem. */
  public IntakeArm() {
    m_motor = new CANSparkMax(CAN_IDs.intakeArm, MotorType.kBrushless);
    m_motor.setInverted(IntakeArmConstants.kArmInverted);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(IntakeArmConstants.kCurrentLimit);
    m_encoder = m_motor.getEncoder();
    m_encoder.setPositionConversionFactor(IntakeArmConstants.kPositionFactor);
    m_encoder.setVelocityConversionFactor(IntakeArmConstants.kVelocityFactor);
    // The starting position is set as 0 in the constructor.
    // This should be done only once for the Arm!
    m_encoder.setPosition(0);

    SmartDashboard.putData(this);
  }

  private double getCurAngle() {
    return m_encoder.getPosition();
  }

  private boolean armIsAtAngle(double armRadians) {
    return (Math.abs(getCurAngle() - armRadians) < IntakeArmConstants.kToleranceRadians);
  }

  private boolean armIsBelowAngle(double armRadians) {
    return (getCurAngle() <= armRadians);
  }

  private boolean armIsAboveAngle(double armRadians) {
    return (getCurAngle() >= armRadians);
  }

  /**
   * rotateClockwise command factory method.
   *
   * @return a command
   */
  public CommandBase rotateToAngleClockwise(double armRadians) {
    // Inline construction of command goes here.
    // Subsystem::run implicitly requires `this` subsystem.
    return this.run(() -> m_motor.set(IntakeArmConstants.kArmUpSpeed))
                .unless(() -> armIsAboveAngle(armRadians)) // Arm is already above the given angle
                .until(() -> armIsAtAngle(armRadians) || armIsAboveAngle(armRadians) || armIsNotsafe())
                .finallyDo((interrupted) -> stop())
                .withName("Rotate Up (Clockwise) To Angle");
  }

  public CommandBase rotateToAngleCounterClockwise(double armRadians) {
    return this.run(() -> m_motor.set(IntakeArmConstants.kArmDownSpeed))
                .unless(() -> armIsBelowAngle(armRadians)) // Arm is already below the given angle
                .until(() -> armIsBelowAngle(armRadians) || armIsBelowAngle(armRadians) || armIsNotsafe())
                .finallyDo((interrupted) -> stop())
                .withName("Rotate Down (Counter Clockwise) to Angle");
  }

  public CommandBase rotateToAngle(double armRadians) {
    return new ConditionalCommand(rotateToAngleClockwise(armRadians), 
                                  rotateToAngleCounterClockwise(armRadians), 
                                  () -> armIsBelowAngle(armRadians))
                .unless(() -> armIsNotsafe())
                .finallyDo((interrupted) -> stop())
                .withName("rotateToAngle");
  }

  /**
   * turnUp command factory method.
   *
   * @return a command
   */
  public CommandBase turnUp() {
    // Inline construction of command goes here.
    // Subsystem::runOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(IntakeArmConstants.kArmUpSpeed))
            .unless(() -> this.armIsNotsafe());
  }

  /**
   * turnDown command factory method.
   *
   * @return a command
   */
  public CommandBase turnDown() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(IntakeArmConstants.kArmDownSpeed))
              .unless(() -> this.armIsNotsafe());
  }

  private void stop() {
    m_motor.set(0);
  }

  /**
   * stop command factory method.
   *
   * @return a command
   */
  public CommandBase stopCmd() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(0.0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean armIsNotsafe() {
    return armIsBelowAngle(IntakeArmConstants.kArmLowerLimit) || 
            armIsAboveAngle(IntakeArmConstants.kArmUpperLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Display the current Arm Position on Smart Dashboard
    double curPos = m_encoder.getPosition();
    SmartDashboard.putNumber("Arm Position", curPos);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    double curPos = m_encoder.getPosition();
    SmartDashboard.putNumber("Arm Position", curPos);
  }
}

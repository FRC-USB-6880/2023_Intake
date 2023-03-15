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

    SmartDashboard.putData(this);
  }

  /**
   * rotateClockwise command factory method.
   *
   * @return a command
   */
  public CommandBase rotateToAngleClockwise(double armDegrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(IntakeArmConstants.kArmUpSpeed))
                .until(() -> m_encoder.getPosition() >= Units.degreesToRadians(armDegrees) || this.armIsUnsafe())
                .finallyDo((interrupted) -> m_motor.set(0.0))
                .withName("Rotate Up To Angle");
  }

  public CommandBase rotateToAngleCounterClockwise(double armDegrees) {
    return this.runOnce(() -> m_motor.set(IntakeArmConstants.kArmDownSpeed))
              .until(() -> m_encoder.getPosition() <= Units.degreesToRadians(armDegrees) || this.armIsUnsafe())
              .finallyDo((interrupted) -> m_motor.set(0.0))
              .withName("Rotate Down to Angle");
  }

  /**
   * turnUp command factory method.
   *
   * @return a command
   */
  public CommandBase turnUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(IntakeArmConstants.kArmUpSpeed))
            .unless(() -> this.armIsUnsafe());
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
              .unless(() -> this.armIsUnsafe());
  }

  /**
   * stop command factory method.
   *
   * @return a command
   */
  public CommandBase stop() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(() -> m_motor.set(0.0));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean armIsUnsafe() {
    // Query some boolean state, such as a digital sensor.
    double curArmPos = m_encoder.getPosition();
    if ((curArmPos  < IntakeArmConstants.kArmLowerLimit) ||
      (curArmPos > IntakeArmConstants.kArmUpperLimit))
      return true;
    else
      return false;
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

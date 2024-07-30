// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class FlywheelIOSparkMax implements FlywheelIO {
  private static final double GEAR_RATIO = 0.5; // changed 6/8/24 derrick

  public final CANSparkMax right = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax left = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = right.getEncoder();
  private final RelativeEncoder leftEncoder = left.getEncoder();
  public final SparkPIDController rightpid = right.getPIDController();
  public final SparkPIDController leftpid = left.getPIDController();

  public FlywheelIOSparkMax() {
    right.restoreFactoryDefaults();
    left.restoreFactoryDefaults();

    right.setCANTimeout(250);
    left.setCANTimeout(250);

    right.setInverted(false);
    left.setInverted(true);
    // left.follow(right, true); // delete this if we are not in follower mode

    right.enableVoltageCompensation(12.0);
    right.setSmartCurrentLimit(80);
    left.enableVoltageCompensation(12.0);
    left.setSmartCurrentLimit(80);

    right.burnFlash();
    left.burnFlash();
  }

  @Override
  public void updateRightInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = right.getAppliedOutput() * right.getBusVoltage();
  }

  public void updateLeftInputs(FlywheelIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
  }

  @Override
  public void setRightVoltage(double volts) {
    right.setVoltage(volts);
  }

  @Override
  public void setLeftVoltage(double volts) {
    left.setVoltage(volts);
  }

  @Override
  public void setRightVelocity(double velocityRadPerSec, double ffVolts) {
    rightpid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setLeftVelocity(double velocityRadPerSec, double ffVolts) {
    leftpid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    right.stopMotor();
    left.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    rightpid.setP(kP, 0);
    rightpid.setI(kI, 0);
    rightpid.setD(kD, 0);
    rightpid.setFF(0 /* .0002*/, 0); // / tune this?
    leftpid.setP(kP, 0);
    leftpid.setI(kI, 0);
    leftpid.setD(kD, 0);
    leftpid.setFF(0 /* .0002*/, 0); // / tune this?
  }

  public double getP() {
    return rightpid.getP();
  }
}

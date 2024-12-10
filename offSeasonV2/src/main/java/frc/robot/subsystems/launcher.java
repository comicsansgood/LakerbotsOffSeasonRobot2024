// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class launcher extends SubsystemBase {
  public CANSparkMax rightLauncherMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax leftLauncherMotor = new CANSparkMax(15, MotorType.kBrushless);
  public SparkPIDController rightLauncherVelocityController = rightLauncherMotor.getPIDController();
  public SparkPIDController leftLauncherVelocityController = leftLauncherMotor.getPIDController();


  /** Creates a new ExampleSubsystem. */
  public launcher() {

    rightLauncherVelocityController.setP(1);
    leftLauncherVelocityController.setP(1);

    rightLauncherMotor.restoreFactoryDefaults();
    leftLauncherMotor.restoreFactoryDefaults();

    rightLauncherMotor.setCANTimeout(250);
    leftLauncherMotor.setCANTimeout(250);

    rightLauncherMotor.setInverted(false);
    leftLauncherMotor.setInverted(true);

    rightLauncherMotor.enableVoltageCompensation(12.0);
    rightLauncherMotor.setSmartCurrentLimit(80);
    leftLauncherMotor.enableVoltageCompensation(12.0);
    leftLauncherMotor.setSmartCurrentLimit(80);

    rightLauncherMotor.burnFlash();
    leftLauncherMotor.burnFlash();
  }
// the launcher motor can now go! -ayden
  public void launcherGo(double speed) {
    leftLauncherMotor.set(speed);
    rightLauncherMotor.set(speed);
  }


  public Command launcherSetVelocity(double leftV, double rightV){
    return runOnce(()-> {
    leftLauncherVelocityController.setReference(leftV, ControlType.kSmartVelocity);
    rightLauncherVelocityController.setReference(rightV, ControlType.kSmartVelocity);
    });
  }

  public Command launcherUptoSpeed(double leftV, double rightV){
    return runOnce(()-> {
    launcherSetVelocity(leftV, rightV);
    Commands.waitSeconds(0.75);
    });
  }


  public Command exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}

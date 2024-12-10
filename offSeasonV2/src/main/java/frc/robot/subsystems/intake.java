package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  public CANSparkMax intakeMotor = new CANSparkMax(13, MotorType.kBrushless);
  
  public intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setCANTimeout(250);
    intakeMotor.setInverted(false);
    intakeMotor.enableVoltageCompensation(12.0);
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.burnFlash();

  }

public Command intakeGo(double speed) {
  return runOnce(() ->intakeMotor.set(speed));


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

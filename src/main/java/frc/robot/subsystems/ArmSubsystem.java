package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private WPI_VictorSPX m_motor;

  private double m_up = 1.0;
  private double m_down = -1.0;


  public ArmSubsystem() {
    // create a new SPARK MAX and configure it
    m_motor = new  WPI_VictorSPX(11);

  }

public Command up() {
    return startEnd(
        () -> {m_motor.set(m_up);},
        () -> {m_motor.set(0);}
    );
}

public Command down() {
        return startEnd(
        () -> {m_motor.set(m_down);},
        () -> {m_motor.set(0);}
    );
}
}
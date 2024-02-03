package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase{

    CANSparkMax m_intakeMotor= new CANSparkMax(8,MotorType.kBrushless);
    public IntakeSubsystem (){
        m_intakeMotor.set(.1);
        
    }

    public void periodic(){
        double motorSpeed = m_intakeMotor.getEncoder().getVelocity();
        SmartDashboard.putNumber("motor speed", motorSpeed);
    }
    
    public double getMotorSpeed(){
        double motorSpeed = m_intakeMotor.getEncoder().getVelocity();
        return motorSpeed; 
    }

    public void setMotorSpeed(double x){
        m_intakeMotor.set(x);
    }
}

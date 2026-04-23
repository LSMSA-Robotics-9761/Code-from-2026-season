package frc.robot.Commands.Shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flywheel extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(9, MotorType.kBrushless); 
    private Voltage motorspeed = Volts.zero();
    public Command setMotor (Voltage speed){
        return runOnce(()-> motorspeed = speed) ;
    }
    @Override
    public void periodic (){
        m_motor.setVoltage(motorspeed);
    }
}


package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {

    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors; 
    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    public final AHRS Gyro;
    private int leftVoltage = 0;
    private int rightVoltage = 0;     
    private final DoubleSolenoid gearShift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);

    
    public Drivetrain() {
        // Add Motors to list
        leftMotors =
                new CANSparkMax[] {
                    new CANSparkMax(10, MotorType.kBrushless),
                    new CANSparkMax(11, MotorType.kBrushless),
                    new CANSparkMax(12, MotorType.kBrushless)
                };
         
        rightMotors = 
                new CANSparkMax[] {
                    new CANSparkMax(13, MotorType.kBrushless),
                    new CANSparkMax(14, MotorType.kBrushless),
                    new CANSparkMax(15, MotorType.kBrushless)
                };
        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2,3);
        Gyro = new AHRS(I2C.Port.kMXP);
    }
    public double getAngle(){
        return Gyro.getAngle();
    }
    public void SetVoltage(){
        //this shit aint right gang
        rightMotors[0].setVoltage(rightVoltage);
        rightMotors[1].setVoltage(rightVoltage);
        rightMotors[2].setVoltage(rightVoltage);
        leftMotors[0].setVoltage(leftVoltage);
        leftMotors[1].setVoltage(leftVoltage);
        leftMotors[2].setVoltage(leftVoltage);
    }
public void setGearHigh() {
    gearShift.set(Value.kReverse);
}
public double getLeftDistance(){
    return leftEncoder.getDistance();
}
public double getRightDistance(){
    return rightEncoder.getDistance();
}
public void setGearLow() {
    gearShift.set(Value.kForward); 
}
public double getLeftVoltage(){
    return leftMotors[1].getAppliedOutput();
}
public double getRightVoltage(){
    return rightMotors[1].getAppliedOutput();
}
public void tankDriveVolts(double left, double right){
    for (MotorController motor : leftMotors) {
        motor.setVoltage(leftVoltage);
    }

    for (MotorController motor : rightMotors) {
        motor.setVoltage(rightVoltage);
    }
}

@Override
public void periodic(){
    SmartDashboard.putNumber("Drivetrain / rightMotor", getRightVoltage());
    SmartDashboard.putNumber("Drivetrain / leftMotor", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain / Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Drivetrain / Right Distance", getRightDistance());
    SmartDashboard.putNumber("Drivetrain / Angle Measure", getAngle());
}
}



        
    


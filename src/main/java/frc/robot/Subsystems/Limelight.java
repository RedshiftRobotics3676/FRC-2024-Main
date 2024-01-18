// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  double tx;
  double ty;
  double ta;

  private double x;
  private double y;
  private double z;
  private double roll;
  private double pitch;
  private double yaw;


  AnalogInput input = new AnalogInput(3);

  // I2C.Port i2cPort =  I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  
  /** Creates a new Limelight. */
  public Limelight() {
    input.setAverageBits(10);
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    return ta;
  }

  @Override
  public void periodic() {
    /* NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    
    // Read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0); */
    
      
    /* tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(-1);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

    // Post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", tx);
    SmartDashboard.putNumber("LimelightY", ty);
    SmartDashboard.putNumber("LimelightArea", ta);
    SmartDashboard.putNumber("LimelightID", tid); */

    double[] bbb = {1,2,3,4,5,6};
      
      
    double[] arr = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(bbb);
      
    x = arr[0];
    y = arr[1];
    z = arr[2];
    roll = arr[3];
    pitch = arr[4];
    yaw = arr[5];
      
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Z", z);
    SmartDashboard.putNumber("Limelight Roll", roll);
    SmartDashboard.putNumber("Limelight Pitch", pitch);
    SmartDashboard.putNumber("Limelight Yaw", yaw);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);

    SmartDashboard.putNumber("Ultrasonic (ft)", input.getAverageVoltage() * 3.30322581/* 1024 * 5 / 1550 */); // Roughly scaled to 1 foot
  
    SmartDashboard.putNumber("colorProx", colorSensor.getProximity());
    SmartDashboard.putString("color", colorSensor.getColor().toString());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightAprilTag extends SubsystemBase {
  /** Creates a new LimelightAprilTag. */
  NetworkTable redBeam = NetworkTableInstance.getDefault().getTable("limelight-MAIN");

  public static HttpCamera limeLightMain;

  private boolean m_visionMode;

  public LimelightAprilTag() {
    limeLightMain = new HttpCamera("limeLightMain", "http://limelight.main:5809/stream.mjpg");
    limeLightMain.setResolution(320, 240);
    limeLightMain.setFPS(90);


    CameraServer.addCamera(limeLightMain);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getXRear(){
    return redBeam.getEntry("tx").getDouble(0.0);
  }

  public double getYRear(){
    return redBeam.getEntry("ty").getDouble(0.0);
  }

  public double getAreaRear(){
    return redBeam.getEntry("ta").getDouble(0.0);
  }


  public boolean hasTargetRear(){
    return redBeam.getEntry("tv").getDouble(0.0) == 1;
  }

  public int getIDRear(){
    return (int)redBeam.getEntry("tid").getDouble(0.0);
  }

  public double getLatPipRear(){
    return redBeam.getEntry("tl").getDouble(0.0)/1000.0;
  }

  public double getLatCapRear(){
    return redBeam.getEntry("cl").getDouble(0.0)/1000.0;
  }

  
  public Pose3d getBotPose(){
    double[] pose = redBeam.getEntry("botpose").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseRed(){
    double[] pose = redBeam.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }
  public Pose3d getBotPoseBlue(){
    double[] pose = redBeam.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    return new Pose3d(new Translation3d(pose[0], pose[1], pose[2]), new Rotation3d(pose[3], pose[4], pose[5]));
  }

  public Transform3d getTransform(){
    return new Transform3d(new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), getBotPose());
  }

  public double getLastEntryTimeStamp(){
    return Timer.getFPGATimestamp() - getLatCapRear() - getLatPipRear();
  }

  /**
   * @param piplineNumber driver = 0, aprilTags = 1, retroreflective = 2
   */
  public void setPipelineRear(int pipelineNumber){
    Number numObj = (Number)pipelineNumber;
    redBeam.getEntry("pipeline").setNumber(numObj);
  }

  
  /**
   * @param piplineNumber 0 = april tags
   */
  public void setPipelineFront(int pipelineNumber){
    Number numObj = (Number)pipelineNumber;
    redBeam.getEntry("pipeline").setNumber(numObj);
  }


  public boolean inVisionMode(){
    return m_visionMode;
  }
  public void setVisionModeOn(){
    m_visionMode = true;
  }
  public void setVisionModeOff(){
    m_visionMode = false;
  }
}


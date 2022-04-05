// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Limelight {
   // private static NetworkTable limelight;
/*
    public Limelight() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }
*/
    public void setLedMode(int num) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(num);
    }
    public void setCamMode(int num) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(num);
    }
    public void selectPipeline(int num) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(num);
    }
    public static boolean isTarget() {
		return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
	}
    public static double getTx() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    public static double getTy() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
    public static double getTa() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
    public static double getTv() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    }
    public static double calcDistance() {
        //((inches) target height - limelight height) / tan(limelight mounting angle + vertical offset by limelight)
        double vertOffset = getTy();
        //84 - 18 inch
        //98.25 - 47.5
        return ((84 - 18) / Math.tan(Math.toRadians(50+vertOffset)));
    }
}

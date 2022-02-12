// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class PathWeaver {
    public static Trajectory getTrajectory(String path) {
        String realPath = "paths/" + path + ".wpilib.json";
        //DriveToTarget.wpilib.json
        Trajectory newTrajectory = new Trajectory();
        try {
            //"/home/lvuser/deploy/output/paths/" + path + ".wpilib.json"
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(realPath);
          // File trajectoryPath = new File(Filesystem.getDeployDirectory(), "paths/" + path + ".wpilib");
           newTrajectory =  TrajectoryUtil.fromPathweaverJson(trajectoryPath);
           return newTrajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
            return null;
        }
    }
    /*
    public String getFilePath() {
        return FileUtilities.getFilePath();
    }
    */
}

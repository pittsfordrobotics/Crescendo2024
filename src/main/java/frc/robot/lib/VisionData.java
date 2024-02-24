// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class VisionData {
    
    private Pose2d visionPose;
    private double time;
    private Matrix<N3, N1> visionReliability;

    public VisionData(Pose2d visionPose, double time, Matrix<N3, N1> visionReliability) {
        this.visionPose = visionPose;
        this.time = time;
        this.visionReliability = visionReliability;
    }

    public Pose2d getVisionPose() {
        return visionPose;
    }

    public double getTime() {
        return time;
    }

    public Matrix<N3, N1> getVisionReliability() {
        return visionReliability;
    }
}

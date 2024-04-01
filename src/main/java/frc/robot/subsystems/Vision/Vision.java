// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionIO.Pipelines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {

    // Initializatio
    private boolean useVision = true;
    private Consumer<VisionData> visionDataConsumer;
    private Supplier<Rotation2d> gyroangle;

    private final VisionIO[] io;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    public Vision(VisionIO ioLimelight1, VisionIO ioLimelight2, Supplier<Rotation2d> gyroangle,
            Consumer<VisionData> visionDataConsumer) {
        this.visionDataConsumer = visionDataConsumer;
        this.gyroangle = gyroangle;
        io = new VisionIO[] { ioLimelight1, ioLimelight2 };
        FieldConstants.aprilTags.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, 0.0));

        Shuffleboard.getTab("Vision").addBoolean("Is Vison Being Used?", this::usingVision);
        Shuffleboard.getTab("Vision").add("UseVisionToggle", new DisabledInstantCommand(this::useVisionToggle));
    }

    private final VisionIO.VisionIOInputs[] inputs = new VisionIO.VisionIOInputs[] { new VisionIO.VisionIOInputs(),
            new VisionIO.VisionIOInputs() };
    private final String[] camNames = new String[] { VisionConstants.LIMELIGHT1_NAME, VisionConstants.LIMELIGHT2_NAME };

    private Pipelines pipeline = Pipelines.Test;

    public void setUseVision(boolean usevision) {
        this.useVision = usevision;
    }

    public void useVisionToggle() {
        this.useVision = !this.useVision;
    }

    public boolean usingVision() {
        return useVision;
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            // update the inputs from the netwrork tables named camNames[i]
            io[i].updateInputs(inputs[i], camNames[i]);
            // keeps the pipeline always the same
            io[i].setPipeline(pipeline, camNames[i]);
        }
        List<Pose2d> allRobotPoses = new ArrayList<>();

        // exit if boolean
        if (!useVision) {
            return;
        }

        // Pose estimation
        for (int i = 0; i < io.length; i++) {

            // gets the pose
            Pose2d visionCalcPose = inputs[i].pose;

            // if the bot is not connected, or the bot is at the origin, skip
            if (visionCalcPose.equals(new Pose2d()) || !inputs[i].connected) {
                continue;
            }

            // exit if off the field (might be bad)
            if (visionCalcPose.getX() < -VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getY() < -VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN) {
                continue;
            }

            // exit if the gyro does not match the vision
            double gyroAngle = gyroangle.get().getDegrees();
            if (Math.abs(gyroAngle - visionCalcPose.getRotation().getDegrees()) > 5) {
                continue;
            }

            // Vision should not be exited at this point?
            Shuffleboard.getTab("Vision").add("Vision/Pose" + i + "/X", visionCalcPose.getX());
            Shuffleboard.getTab("Vision").add("Vision/Pose" + i + "/Y", visionCalcPose.getY());
            Shuffleboard.getTab("Vision").add("Vision/Pose" + i + "/Theta", visionCalcPose.getRotation().getDegrees());

            // Get tag poses and update last detection times
            List<Pose3d> tagPoses = new ArrayList<>();
            for (int z = 0; z < inputs[i].tagIDs.length; z++) {
                int tagId = (int) inputs[i].tagIDs[z];
                lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) inputs[i].tagIDs[z]);
                tagPose.ifPresent(tagPoses::add);
            }
            Shuffleboard.getTab("Vision").add("TagIds", inputs[i].tagIDs);

            // Gets the average distance to tag
            double avgDistance = inputs[i].avgTagDist;
            Shuffleboard.getTab("Vision").add("Vision/AvgDist", avgDistance);
            Shuffleboard.getTab("Vision").add("Vision/NumTags", inputs[i].tagCount);
            // TODO: Double check this can be over 2 lol

            // exit in auto unless have 2 tags and 2 of them are the good tags under 4
            // meters
            if (DriverStation.isAutonomous() && (inputs[i].tagCount < 2 || avgDistance > 4.0)) {
                continue;
            }

            // Calculate standard deviation to give to the .addVisionData() swerve method
            // Standard Deveation is inverse to confidence level
            double xyStdDev = VisionConstants.XY_STD_DEV_COEF * Math.pow(avgDistance, 2.0)
                    / Math.pow(inputs[i].tagCount, 3.0);
            double thetaStdDev = VisionConstants.THETA_STD_DEV_COEF * Math.pow(avgDistance, 2.0)
                    / Math.pow(inputs[i].tagCount, 3.0);
            Shuffleboard.getTab("Vision").add("Vision/XYstd", xyStdDev);

            // Add vision data to swerve pose estimator
            VisionData visionData = new VisionData(visionCalcPose, inputs[i].captureTimestamp,
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            visionDataConsumer.accept(visionData);

            // Add robot pose from this camera to a list of all robot poses
            allRobotPoses.add(visionCalcPose);
        }
    }
}

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DisabledInstantCommand;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.LimelightHelpers;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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
    private Supplier<Double> robotRotationalVelocity;
    private double xyStdDev = 200;
    private boolean hasGreatSpeakerReading = false;

    private final VisionIO[] io;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private Pipelines pipeline = Pipelines.Test;

    StructArrayPublisher<Pose2d> visionPoseArrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Vision Poses", Pose2d.struct).publish();

    public Vision(VisionIO ioLimelight1, VisionIO ioLimelight2, Supplier<Rotation2d> gyroangle,
            Supplier<Double> robotRotationalVelocity, Consumer<VisionData> visionDataConsumer) {
        this.visionDataConsumer = visionDataConsumer;
        this.gyroangle = gyroangle;
        this.robotRotationalVelocity = robotRotationalVelocity;
        io = new VisionIO[] { ioLimelight1, ioLimelight2 };
        FieldConstants.aprilTags.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, 0.0));

        Shuffleboard.getTab("Vision").addBoolean("Is Vison Being Used?", this::usingVision);
        Shuffleboard.getTab("Vision").add("UseVisionToggle", new DisabledInstantCommand(this::useVisionToggle));
        
        for (int i = 0; i < io.length; i++) {
            int number = i; // why are ints dumb
            Shuffleboard.getTab("Vision").addDouble(i + "/AvgTagDist", () -> this.inputs[number].avgTagDist);
            Shuffleboard.getTab("Vision").addInteger(i + "/NumTags", () -> this.inputs[number].tagCount);
            Shuffleboard.getTab("Vision").addDoubleArray(i + "/TagDistances", () -> this.inputs[number].tagDistances);
            Shuffleboard.getTab("Vision").addString(i + "/TagIDs", () -> this.inputs[number].tagIDs.toString()); //why are ints dumb
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_X", () -> this.inputs[number].pose.getX());
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_Y", () -> this.inputs[number].pose.getY());
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_Theta", () -> this.inputs[number].pose.getRotation().getDegrees());
        }
        Shuffleboard.getTab("Vision").addDouble("XY_std", this::getXYstdDev);
        Shuffleboard.getTab("Vision").addBoolean("Has Great Speaker Reading", this::hasGreatSpeakerReading);
    }

    private final VisionIO.VisionIOInputs[] inputs = new VisionIO.VisionIOInputs[] { new VisionIO.VisionIOInputs(),
            new VisionIO.VisionIOInputs() };
    private final String[] camNames = new String[] { VisionConstants.LIMELIGHT1_NAME, VisionConstants.LIMELIGHT2_NAME };

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

    public double getXYstdDev() {
        return xyStdDev;
    }

    public boolean hasGreatSpeakerReading() {
        return hasGreatSpeakerReading;
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            // update the inputs from the netwrork tables named camNames[i]
            io[i].updateInputs(inputs[i], camNames[i], gyroangle.get().getDegrees());
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
                System.out.println("Gyro and Vision do not match");
                // continue;
            }

            // exit if the robot is rotating too fast
            if (Math.abs(robotRotationalVelocity.get()) > 6.28) {
                continue;
            }

            if (inputs[i].tagCount == 0) {
                continue;
            }

            // Get tag poses and update last detection times
            List<Pose3d> tagPoses = new ArrayList<>();
            for (int z = 0; z < inputs[i].tagIDs.length; z++) {
                int tagId = (int) inputs[i].tagIDs[z];
                lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) inputs[i].tagIDs[z]);
                tagPose.ifPresent(tagPoses::add);
            }

            // Gets the average distance to tag
            double avgDistance = inputs[i].avgTagDist;
            // TODO: Double check this can be over 2 lol

            // Check if the robot has both speaker tags for red or blue
            boolean hasBlueSpeakerTags = (Arrays.binarySearch(inputs[i].tagIDs, 7) >= 0)
                    && (Arrays.binarySearch(inputs[i].tagIDs, 8) >= 0);
            boolean hasRedSpeakerTags = (Arrays.binarySearch(inputs[i].tagIDs, 3) >= 0)
                    && (Arrays.binarySearch(inputs[i].tagIDs, 4) >= 0);

            // Checks if has supergood reading at the speaker
            hasGreatSpeakerReading = ((inputs[i].tagCount >= 2) && (avgDistance < 4.0)
                    && (hasBlueSpeakerTags || hasRedSpeakerTags));

            // Exits when in auto if it doesnt have a great great reading
            if (DriverStation.isAutonomous() && !hasGreatSpeakerReading) {
                continue;
            }

            // Calculate standard deviation to give to the .addVisionData() swerve method
            // Standard Deveation is inverse to confidence level
            xyStdDev = VisionConstants.XY_STD_DEV_COEF * (avgDistance * avgDistance)
                    / (inputs[i].tagCount * inputs[i].tagCount);
            if (hasGreatSpeakerReading) {
                xyStdDev = xyStdDev * 0.5;
            }
            double thetaStdDev = VisionConstants.THETA_STD_DEV_COEF;

            // Add vision data to swerve pose estimator
            VisionData visionData = new VisionData(visionCalcPose, inputs[i].captureTimestamp,
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            visionDataConsumer.accept(visionData);

            // Add robot pose from this camera to a list of all robot poses
            allRobotPoses.add(visionCalcPose);
            // xyStdDev = 200;
        }
        visionPoseArrayPublisher.set(allRobotPoses.toArray(new Pose2d[0]));
    }
}

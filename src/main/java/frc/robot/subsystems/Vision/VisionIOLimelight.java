package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.util.LimelightHelpers;
import edu.wpi.first.wpilibj.Timer;


public class VisionIOLimelight implements VisionIO {
    public Pose3d lastPose = new Pose3d();

    // This is how it was last year.
    // I changed it so that each method takes a name parameter. So that multiple limelights is possible.
    // I might be completely of base here, but I think this should work.
    // private final String limelightName = "";
    // private final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);

    public VisionIOLimelight(String CamName) {
        setLEDs(LED.OFF, CamName);
        setPipeline(Pipelines.Test, CamName);
    }

    public void updateInputs(VisionIOInputs inputs, String limelightName) {
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);

        NetworkTableEntry heartbeatEntry = limelight.getEntry("hb");
        NetworkTableEntry botposeEntry = DriverStation.getAlliance().orElse
            (DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 
            limelight.getEntry("botpose_wpiblue") : limelight.getEntry("botpose_wpired");
        double pipelineLatency = LimelightHelpers.getLatency_Pipeline(limelightName);
        double captureLatency = LimelightHelpers.getLatency_Capture(limelightName);
        double totalLatency = pipelineLatency + captureLatency; // ms
    
        if (!lastPose.equals(new Pose3d(botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2], new Rotation3d(botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5])))) {
            inputs.captureTimestamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(totalLatency);
            inputs.botXYZ = new double[]{botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2]};
            inputs.botRPY = new double[]{botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5]};
            inputs.tagIDs = new double[]{(int) LimelightHelpers.getFiducialID(limelightName)};
            lastPose = new Pose3d(botposeEntry.getDoubleArray(new double[7])[0], botposeEntry.getDoubleArray(new double[7])[1], botposeEntry.getDoubleArray(new double[7])[2], new Rotation3d(botposeEntry.getDoubleArray(new double[7])[3], botposeEntry.getDoubleArray(new double[7])[4], botposeEntry.getDoubleArray(new double[7])[5]));
        }
        inputs.captureLatency = captureLatency;
        inputs.pipelineLatency = pipelineLatency;
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);
        inputs.connected = heartbeatEntry.getDouble(0.0) > 0.0;
        inputs.vAngle = LimelightHelpers.getTY(limelightName);
        inputs.hAngle = LimelightHelpers.getTX(limelightName);
    }

    @Override
    public void setPipeline(Pipelines pipeline, String limelightName) {
        NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);
        limelight.getEntry("pipeline").setDouble(pipeline.getNum());
    }

    @Override
    public void setCameraModes(CameraMode camera, String limelightName) {
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);
        limelight.getEntry("camMode").setDouble(camera.getNum());
    }

    @Override
    public void setLEDs(LED led, String limelightName) {
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);
        limelight.getEntry("ledMode").setDouble(led.getNum());
    }
}
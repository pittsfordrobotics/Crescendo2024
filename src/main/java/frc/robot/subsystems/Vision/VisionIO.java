package frc.robot.subsystems.Vision;

/** Vision subsystem hardware interface. */
public interface VisionIO {
    class VisionIOInputs {
        public double pipelineLatency = 0.0;
        public double captureLatency = 0.0;
        public double captureTimestamp = 0.0;
        public boolean hasTarget = false;
        public boolean connected = false;
        public double vAngle = 0.0;
        public double hAngle = 0.0;
        public double[] tagIDs = new double[]{};
        public double[] botXYZ = new double[]{};
        public double[] botRPY = new double[]{};
    }

    /**
     * Far: has the lowest FPS, but allows for possible vision updates when halfway down the field
     * Mid: has slightly higher FPS, but allows for farther vision updates when halfway down the field
     * Close: has the highest FPS, but only has limited range
     * Retro: uses the green lights
     */

    
    enum Pipelines {
        Test(0), Close(1), Mid(2), Far(3), Retro(4);

        private final int num;
        Pipelines(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum LED {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int num;

        LED(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum CameraMode {
        VISION_PROCESSING(0), DRIVER_CAMERA(1);

        private final int num;

        CameraMode(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(VisionIOInputs inputs, String CamName) {}

    /** Enabled or disabled vision LEDs. */
    default void setLEDs(LED led, String CamName) {}

    /** Enabled or disabled vision LEDs. */
    default void setCameraModes(CameraMode mode, String CamName) {}

    /** Sets the pipeline number. */
    default void setPipeline(Pipelines pipeline, String CamName) {}

}
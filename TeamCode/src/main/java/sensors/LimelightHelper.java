package sensors;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import org.firstinspires.ftc.teamcode.INTESTING3.blueteleop;
import org.firstinspires.ftc.teamcode.control1.PIDConfig;


public class LimelightHelper {

    // ==========================
    // VARIABLES
    // ==========================
    private final Limelight3A limelight;
    private final IMUHelper imu;
    private final PIDConfig pidDistance;
    private final PIDConfig pidAngle;

    private double lastDistanceError, distanceIntegral, lastAngleError, angleIntegral = 0;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private blueteleop.PIDController distancePID;
    private blueteleop.PIDController yawPID;

    // ==========================
    // CONSTRUCTOR
    // ==========================

    public LimelightHelper(HardwareMap hardwareMap, String cameraName,
                           String imuName,
                           RevHubOrientationOnRobot.UsbFacingDirection usbDir,
                           RevHubOrientationOnRobot.LogoFacingDirection logoDir,
                           PIDConfig pidAngle, PIDConfig pidDistance) {

        this.limelight = hardwareMap.get(Limelight3A.class, cameraName);
        this.imu = new IMUHelper(hardwareMap, imuName, usbDir, logoDir);

        this.pidAngle = pidAngle;
        this.pidDistance = pidAngle;
    }


    // ==========================
    // LIMELIGHT DATA ACCESS
    // ==========================

    /** Returns the horizontal offset from the target (tx) */
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    /** Returns the vertical offset from the target (ty) */
    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    /** Returns the detected target area (ta) */
    public double getTa() {
        return limelight.getLatestResult().getTa();
    }

    /** Checks if there is a valid target */
    public boolean isTargetValid() {
        return limelight.getLatestResult().isValid();
    }

    /** Returns the robot’s 3D pose (MT1) */
    public Pose3D getBotPoseMT1() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) return r.getBotpose();
        return null;
    }

    /** Returns the robot’s 3D pose (MT2) using IMU orientation */
    public Pose3D getBotPoseMT2() {
        updateRobotOrientation();
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) return r.getBotpose_MT2();
        return null;
    }

    /** Returns a list of detected fiducials (AprilTags) */
    public List<LLResultTypes.FiducialResult> getFiducials() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) return result.getFiducialResults();
        return null;
    }

    /** Returns the distance to a given fiducial */
    public double getDistanceToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            return Math.sqrt(
                    robotPose.getPosition().x * robotPose.getPosition().x +
                            robotPose.getPosition().y * robotPose.getPosition().y +
                            robotPose.getPosition().z * robotPose.getPosition().z
            );
        }
        return -1;
    }

    /** Returns the yaw (horizontal angle) to the fiducial */
    public double getYawToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            return Math.toDegrees(Math.atan2(robotPose.getPosition().y, robotPose.getPosition().x));
        }
        return 0;
    }

    /** Returns the pitch (vertical angle) to the fiducial */
    public double getPitchToFiducial(LLResultTypes.FiducialResult fiducial) {
        if (fiducial != null) {
            Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
            double horizontalDist = Math.sqrt(
                    robotPose.getPosition().x * robotPose.getPosition().x +
                            robotPose.getPosition().y * robotPose.getPosition().y
            );
            return Math.toDegrees(Math.atan2(robotPose.getPosition().z, horizontalDist));
        }
        return 0;
    }

    /** Returns the estimated robot position on the field */
    public Pose3D getEstimatedFieldPosition() {
        updateRobotOrientation();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D mt2 = result.getBotpose_MT2();
            if (mt2 != null) return mt2;
            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) return mt1;
        }
        return null;
    }


    // ==========================
    // TELEMETRY
    // ==========================

    /** Sends full positional and fiducial telemetry */
    public void telemetryFull(Telemetry telemetry) {
        Pose3D fieldPose = getEstimatedFieldPosition();
        if (fieldPose != null) {
            telemetry.addData("Robot (Field)", "X: %.2f Y: %.2f Z: %.2f",
                    fieldPose.getPosition().x,
                    fieldPose.getPosition().y,
                    fieldPose.getPosition().z);
        }

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult f : fiducials) {
                double dist = getDistanceToFiducial(f);
                double yaw = getYawToFiducial(f);
                double pitch = getPitchToFiducial(f);
                telemetry.addData("Fiducial " + f.getFiducialId(),
                        String.format("Dist: %.2fm Yaw: %.1f° Pitch: %.1f°", dist, yaw, pitch));
            }
        }
    }


    // ==========================
    // MOVEMENT CALCULATIONS
    // ==========================

    /**
     * Calculates the direction and speed required to move to a target field position.
     * @return array [vForward, vStrafe, vRot]
     */
    public double[] calculateMovementToPosition(double xTarget, double yTarget, double dtSeconds) {
        Pose3D currentPose = getEstimatedFieldPosition();
        if (currentPose == null) return new double[]{0, 0, 0};

        double dx = xTarget - currentPose.getPosition().x;
        double dy = yTarget - currentPose.getPosition().y;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double yawError = imu.normalizeAngle(angleToTarget - imu.getYaw());

        // PID - distance
        distanceIntegral += distance * dtSeconds;
        double distanceDerivative = (distance - lastDistanceError) / dtSeconds;
        lastDistanceError = distance;
        double vForward = pidDistance.kP * distance +
                pidDistance.kI * distanceIntegral +
                pidDistance.kD * distanceDerivative;

        // PID - rotation
        angleIntegral += yawError * dtSeconds;
        double angleDerivative = (yawError - lastAngleError) / dtSeconds;
        lastAngleError = yawError;
        double vRot = pidAngle.kP * yawError +
                pidAngle.kI * angleIntegral +
                pidAngle.kD * angleDerivative;

        // Clamp outputs
        vForward = Math.min(vForward, 1);
        vRot = Math.max(Math.min(vRot, 1), -1);

        return new double[]{vForward, 0, vRot}; // [forward, strafe (0 for tank), rotation]
    }


    // ==========================
    // LIMELIGHT STATUS
    // ==========================

    /** Returns the time since the last update (ms) */
    public long getTimeSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }

    /** Checks if Limelight is connected */
    public boolean isConnected() {
        return limelight.isConnected();
    }


    // ==========================
    // PYTHON SNAP METHODS
    // ==========================

    /** Updates Python SnapScript inputs (8 values) */
    public boolean updatePythonInputs(double i1, double i2, double i3, double i4,
                                      double i5, double i6, double i7, double i8) {
        return limelight.updatePythonInputs(i1, i2, i3, i4, i5, i6, i7, i8);
    }

    /** Updates Python SnapScript inputs using an array */
    public boolean updatePythonInputs(double[] inputs) {
        return limelight.updatePythonInputs(inputs);
    }


    // ==========================
    // PIPELINE METHODS
    // ==========================

    /** Switches to the desired pipeline by index */
    public boolean switchPipeline(int index) {
        return limelight.pipelineSwitch(8);
    }

    /** Reloads the current pipeline */
    public boolean reloadPipeline() {
        return limelight.reloadPipeline();
    }

    /** Captures a snapshot */
    public boolean captureSnapshot(String name) {
        return limelight.captureSnapshot(name);
    }

    /** Deletes a snapshot */
    public boolean deleteSnapshot(String name) {
        return limelight.deleteSnapshot(name);
    }


    // ==========================
    // FIELD / ORIENTATION METHODS
    // ==========================

    /** Updates robot orientation for MegaTag2 */
    private void updateRobotOrientation() {
        double yaw = imu.getYaw();
        limelight.updateRobotOrientation(yaw);
    }

    /** Uploads a field map */
    public boolean uploadFieldmap(LLFieldMap map, Integer index) {
        return limelight.uploadFieldmap(map, index);
    }


    // ==========================
    // POLLING CONTROL
    // ==========================

    public void start() { limelight.start(); }
    public void pause() { limelight.pause(); }
    public void stop() { limelight.stop(); }
    public boolean isRunning() { return limelight.isRunning(); }

    /** Sets the polling rate in Hz */
    public void setPollRateHz(int hz) {
        limelight.setPollRateHz(hz);
    }


    // ==========================
    // LATENCY / PERFORMANCE INFO
    // =========================xxxxx=

    /** Returns the capture latency (ms) */
    public double getCaptureLatency() {
        return limelight.getLatestResult().getCaptureLatency();
    }

    /** Returns the target processing latency (ms) */
    public double getTargetingLatency() {
        return limelight.getLatestResult().getTargetingLatency();
    }

    /** Returns the result parsing latency (ms) */
    public double getParseLatency() {
        return limelight.getLatestResult().getParseLatency();
    }


    // ==========================
    // FIDUCIAL UTILITIES
    // ==========================

    /** Returns the ID of the closest fiducial */
    public Integer getClosestFiducialID() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult closest = fiducials.get(0);
        double minDist = getDistanceToFiducial(closest);
        for (LLResultTypes.FiducialResult f : fiducials) {
            double dist = getDistanceToFiducial(f);
            if (dist < minDist) {
                minDist = dist;
                closest = f;
            }
        }
        return closest.getFiducialId();
    }


    // ==========================
    // MOVEMENT SETUP & CONTROL
    // ==========================

    /** Configures the motors and PID controllers for movement */
    public void setupMovement(DcMotor leftMotor, DcMotor rightMotor, PIDConfig distanceConfig, PIDConfig yawConfig) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.distancePID = new blueteleop.PIDController(distanceConfig);
        this.yawPID = new blueteleop.PIDController(yawConfig);
    }

    /**
     * Moves the robot (tank drive) toward a fiducial until it reaches the desired distance.
     * @return true if target distance is reached
     */
    public boolean moveToFiducialByIdTank(int targetID, double desiredDistance, double maxPower, Telemetry telemetry) {
        if (leftMotor == null || rightMotor == null || distancePID == null || yawPID == null) return false;

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null || fiducials.isEmpty()) return false;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetID) { target = f; break; }
        }

        double distance = getDistanceToFiducial(target);
        double distanceError = distance - desiredDistance;

        // Stop when within tolerance
        if (Math.abs(distanceError) < 0.05) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }

        // PID forward
        double forwardPower = distancePID.calculate(desiredDistance, distanceError);
        forwardPower = clamp(forwardPower, -maxPower, maxPower);

        // PID yaw correction
        double yawToTag = getYawToFiducial(target);
        double yawCorrection = yawPID.calculate(0, yawToTag);

        // Apply motor power
        double leftPower = clamp(forwardPower + yawCorrection, -maxPower, maxPower);
        double rightPower = clamp(forwardPower - yawCorrection, -maxPower, maxPower);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Fiducial ID", target.getFiducialId());
            telemetry.addData("Distance Error", distanceError);
            telemetry.addData("Forward Power", forwardPower);
            telemetry.addData("Yaw To Tag", yawToTag);
            telemetry.addData("Yaw Correction", yawCorrection);
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }

        return false;
    }

    /**
     * Moves the robot (mecanum drive) toward a fiducial until it reaches the desired distance.
     * @return true if target distance is reached
     */
    public boolean moveToFiducialByIdMecanum(int targetID, DcMotor frontLeft, DcMotor frontRight,
                                             DcMotor backLeft, DcMotor backRight,
                                             double desiredDistance, double maxPower, Telemetry telemetry) {

        if (frontLeft == null || frontRight == null || backLeft == null || backRight == null ||
                distancePID == null || yawPID == null)
            return false;

        List<LLResultTypes.FiducialResult> fiducials = getFiducials();
        if (fiducials == null || fiducials.isEmpty()) return false;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetID) { target = f; break; }
        }
        if (target == null) return false;

        double distance = getDistanceToFiducial(target);
        double distanceError = distance - desiredDistance;
        double yawToTag = getYawToFiducial(target);

        // Stop when within tolerance
        if (Math.abs(distanceError) < 0.05) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return true;
        }

        // PID distance and yaw
        double forwardPower = clamp(distancePID.calculate(desiredDistance, distanceError), -maxPower, maxPower);
        double yawCorrection = yawPID.calculate(0, yawToTag);

        // Convert polar to Cartesian for mecanum drive
        double strafePower = forwardPower * Math.sin(Math.toRadians(yawToTag));
        double forwardComponent = forwardPower * Math.cos(Math.toRadians(yawToTag));

        // Calculate motor powers
        double fl = clamp(forwardComponent + strafePower + yawCorrection, -maxPower, maxPower);
        double fr = clamp(forwardComponent - strafePower - yawCorrection, -maxPower, maxPower);
        double bl = clamp(forwardComponent - strafePower + yawCorrection, -maxPower, maxPower);
        double br = clamp(forwardComponent + strafePower - yawCorrection, -maxPower, maxPower);

        // Apply power
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Fiducial ID", target.getFiducialId());
            telemetry.addData("Distance Error", distanceError);
            telemetry.addData("Forward", forwardComponent);
            telemetry.addData("Strafe", strafePower);
            telemetry.addData("Yaw Correction", yawCorrection);
            telemetry.addData("FL", fl);
            telemetry.addData("FR", fr);
            telemetry.addData("BL", bl);
            telemetry.addData("BR", br);
            telemetry.update();
        }

        return false;
    }


    // ==========================
    // UTILITY
    // ==========================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
package sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.control1.PIDConfig;

public class LimelightDriveController {

    private final LimelightHelper limelightHelper;
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;

    private final double maxPower;

    /**
     * @param hardwareMap HardwareMap
     * @param cameraName nome do Limelight
     * @param imuName nome do IMU
     * @param leftMotor left drive
     * @param rightMotor right drive
     * @param usbDir USB Facing
     * @param logoDir Logo Facing
     * @param pidDistance PID para distância
     * @param pidAngle PID para rotação
     * @param maxPower máxima potência de saída (0-1)
     */
    public LimelightDriveController(HardwareMap hardwareMap,
                                    String cameraName,
                                    String imuName,
                                    DcMotor leftMotor,
                                    DcMotor rightMotor,
                                    RevHubOrientationOnRobot.UsbFacingDirection usbDir,
                                    RevHubOrientationOnRobot.LogoFacingDirection logoDir,
                                    PIDConfig pidDistance,
                                    PIDConfig pidAngle,
                                    double maxPower) {
        this.limelightHelper = new LimelightHelper(
                hardwareMap, cameraName, imuName, usbDir, logoDir, pidDistance, pidAngle
        );
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.maxPower = maxPower;

        // Configuração inicial
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Moves the robot to the X,Y position in the field
     *
     * @param xTarget target X
     * @param yTarget target Y
     * @param dtSeconds interval since the last loop
     * @param telemetry Optional telemetry
     * @return true if close enough
     */
    public boolean moveToPosition(double xTarget, double yTarget, double dtSeconds, Telemetry telemetry) {
        double[] powers = limelightHelper.calculateMovementToPosition(xTarget, yTarget, dtSeconds);

        // Frente e rotação
        double forward = powers[0];
        double rotation = powers[2];

        // Converte para tank drive
        double leftPower = clamp(forward + rotation, -maxPower, maxPower);
        double rightPower = clamp(forward - rotation, -maxPower, maxPower);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Telemetria opcional
        if (telemetry != null) {
            telemetry.addData("Target", "(%.2f, %.2f)", xTarget, yTarget);
            telemetry.addData("Power", "L: %.2f R: %.2f", leftPower, rightPower);
            telemetry.addData("Forward/Rot", "F: %.2f R: %.2f", forward, rotation);
            telemetry.update();
        }

        // Condição de parada (distância menor que 0.1m)
        Pose3D pose = limelightHelper.getEstimatedFieldPosition();
        if (pose == null) return false;

        double dx = xTarget - pose.getPosition().x;
        double dy = yTarget - pose.getPosition().y;
        double distance = Math.sqrt(dx*dx + dy*dy);

        return distance < 0.1; // chegou
    }

    private double clamp(double val, double min, double max) {
        return Math.max(Math.min(val, max), min);
    }

    /** Para o robô */
    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /** Telemetria completa do Limelight */
    public void telemetry(Telemetry telemetry) {
        limelightHelper.telemetryFull(telemetry);
    }
}
package org.firstinspires.ftc.teamcode.INTESTING3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.control1.PIDConfig;

@TeleOp
public class blueteleop extends LinearOpMode {
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor flmotor = hardwareMap.dcMotor.get("frontleft");
        DcMotor blMotor = hardwareMap.dcMotor.get("backleft");
        DcMotor frmotor = hardwareMap.dcMotor.get("frontright");
        DcMotor brmotor = hardwareMap.dcMotor.get("backright");

        frmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flpower = (y + x + rx) / den;
            double blpower = (y - x + rx) / den;
            double frpower = (y - x - rx) / den;
            double brpower = (y + x - rx) / den;

            flmotor.setPower(flpower);
            frmotor.setPower(frpower);
            blMotor.setPower(blpower);
            brmotor.setPower(brpower);

            if (gamepad1.x) {
                limelight.start();
            }
            if (gamepad1.y) {
                limelight.stop();
            }

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult apriltag : result.getFiducialResults()) {
                    if (apriltag.getFiducialId() == 20) {
                        double tx = result.getTx();  // How far left or right the target is (degrees)
                        double ty = result.getTy(); // How far up or down the target is (degrees)
                        double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                        telemetry.addData("Blue Team Target X", tx);
                        telemetry.addData("Blue Team Target Y", ty);
                        telemetry.addData("Blue Team  Target Area", ta);
                    } else {
                        telemetry.addData("April tag is ", apriltag.getFiducialId());
                        telemetry.update();
                    }
                }
            } else if (result == null) {
                telemetry.addData("Result is null", "");
                telemetry.update();
            }
        }
    }

    public static class PIDController {

        private final PIDConfig config;
        private double integral;
        private double previousError;

        public PIDController(PIDConfig config) {
            this.config = config;
            this.integral = 0;
            this.previousError = 0;
        }

        public double calculate(double setpoint, double measurement) {
            double error = setpoint - measurement;

            if (Math.abs(error) < config.iZone) {
                integral += error;
            } else {
                integral = 0;
            }

            double derivative = error - previousError;
            previousError = error;

            return config.kP * error + config.kI * integral + config.kD * derivative + config.kF * setpoint;
        }

        public void reset() {
            integral = 0;
            previousError = 0;
        }
    }
}
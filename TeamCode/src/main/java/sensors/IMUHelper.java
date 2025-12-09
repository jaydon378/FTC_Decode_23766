package sensors;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUHelper {

    private enum Axis {
        YAW,
        PITCH,
        ROLL
    }

    private final IMU imu;
    private YawPitchRollAngles robotOrientation;


    /**
     * @param hardwareMap Robot HardwareMap
     * @param imuName Name of the IMU configured on the Driver Station
     * @param usbFacingDirection USB FacingDirection of the Control Hub
     * @param logoFacingDirection Logo FacingDirection of the Control Hub
     */
    public IMUHelper(HardwareMap hardwareMap, String imuName,
                     RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection,
                     RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection){

        imu = hardwareMap.get(IMU.class, imuName);

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection
                ));

        imu.initialize(imuParameters);

    }

    /**
     * Normalizes an angle to be between -180 and 180 degrees
     */
    public double normalizeAngle(double angle){
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private double get(Axis axis) {
        switch (axis) {
            case YAW:
                return getYaw();
            case PITCH:
                return getPitch();
            case ROLL:
                return getRoll();
            default:
                return 0;
        }
    }

    public double getYaw(){
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    public double getPitch(){
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
    }

    public double getRoll(){
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    public void update(){
        robotOrientation = imu.getRobotYawPitchRollAngles();
    }

}
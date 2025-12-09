package org.firstinspires.ftc.teamcode.common;

import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.impl.IMUEx;

public class Parts {

    // declaring parts
    public static MotorEx FR, FL, BR, BL;
    public static MotorEx shooter, shooter2;
    public static MotorEx intake;
    public static MotorEx spin;
    public static ServoEx hood;
   // public static IMUEx imu;

    public Parts() {
        FL = new MotorEx("frontleft").reversed().brakeMode();
        FR = new MotorEx("frontright").brakeMode();
        BR = new MotorEx("backright").brakeMode();
        BL = new MotorEx("backleft").reversed().brakeMode();

        shooter = new MotorEx("shooterOne").reversed().brakeMode();
        shooter2 = new MotorEx("shooterTwo").brakeMode();

        intake = new MotorEx("intake").brakeMode();
        spin = new MotorEx("lazysue").reversed().brakeMode();

        hood = new ServoEx("hood");

       // imu = new IMUEx("imu", Direction.UP, Direction.RIGHT).zeroed();
    }
}


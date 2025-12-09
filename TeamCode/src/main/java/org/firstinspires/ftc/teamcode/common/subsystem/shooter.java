package org.firstinspires.ftc.teamcode.common.subsystem;

//import com.bylazar.configurables.annotations.Configurable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.powerable.SetPower;
import dev.nextftc.control.ControlSystem;
import org.firstinspires.ftc.teamcode.common.Parts;

//@Config
public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter() {}

    Parts part = new Parts();
    public static PIDCoefficients coefficients = new PIDCoefficients(0.004, 0, 0);

    public final double velocity = 100.0;
    public final ControlSystem powercontrolled = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(0.003)
            .build();

    // idk if this is good or not
    public final Command off = new RunToVelocity(powercontrolled, 0.0).requires(this);
    public final Command on = new RunToVelocity(powercontrolled, velocity).requires(this);
    public final Command reverse = new RunToVelocity(powercontrolled, -velocity).requires(this);

    @Override
    public void periodic() {
        Parts.shooter.setPower(powercontrolled.calculate(Parts.shooter.getState()));
        Parts.shooter2.setPower(powercontrolled.calculate(Parts.shooter2.getState()));
    }

}

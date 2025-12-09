package org.firstinspires.ftc.teamcode;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.NextFTCOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.SubsystemComponent;
import org.firstinspires.ftc.teamcode.common.subsystem.hood;
import org.firstinspires.ftc.teamcode.common.subsystem.shooter;
import org.firstinspires.ftc.teamcode.common.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.subsystem.Rintake;
import org.firstinspires.ftc.teamcode.common.Parts;

@TeleOp(group = "Decode")
public class twoplayer extends NextFTCOpMode {
    Parts part = new Parts();
    public twoplayer() {
        addComponents(
                new SubsystemComponent(hood.INSTANCE, shooter.INSTANCE, Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                Parts.FL,
                Parts.FR,
                Parts.BL,
                Parts.BR,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
               // new FieldCentric(Parts.imu)
        );
        driverControlled.schedule();

        Gamepads.gamepad2().b().whenBecomesTrue(
                hood.INSTANCE.turn
        );
        Gamepads.gamepad2().a().whenBecomesTrue(
                hood.INSTANCE.reset
        );

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(shooter.INSTANCE.on)
                .whenBecomesFalse(shooter.INSTANCE.off);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(shooter.INSTANCE.reverse)
                .whenBecomesFalse(shooter.INSTANCE.off);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(Intake.INSTANCE.power)
                .whenBecomesFalse(Intake.INSTANCE.stop);

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(Rintake.INSTANCE.power)
                .whenBecomesFalse(Rintake.INSTANCE.stop);


    }

    @Override
    public void onUpdate() {
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}

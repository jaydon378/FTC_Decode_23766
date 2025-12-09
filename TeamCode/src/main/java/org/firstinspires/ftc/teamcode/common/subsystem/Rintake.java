package org.firstinspires.ftc.teamcode.common.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;

import org.firstinspires.ftc.teamcode.common.Parts;

public class Rintake implements Subsystem {
    public static final Rintake INSTANCE = new Rintake();

    private Rintake() {
    }

    Parts part = new Parts();


    public Command power = new SetPower(Parts.intake, -0.5).requires(this);
    public Command stop = new SetPower(Parts.intake, 0).requires(this);


}

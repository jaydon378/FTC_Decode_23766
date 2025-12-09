package org.firstinspires.ftc.teamcode.common.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;

import org.firstinspires.ftc.teamcode.common.Parts;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    Parts part = new Parts();


    public Command power = new SetPower(Parts.intake, 1).requires(this);
    public Command stop = new SetPower(Parts.intake, 0).requires(this);


}
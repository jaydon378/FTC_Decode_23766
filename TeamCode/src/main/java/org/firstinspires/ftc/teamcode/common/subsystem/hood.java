package org.firstinspires.ftc.teamcode.common.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.positionable.SetPosition;
import org.firstinspires.ftc.teamcode.common.Parts;

public class hood implements Subsystem {
    public static final hood INSTANCE = new hood();
    private hood() {}

    Parts part = new Parts();

    // idk names for these
    public Command turn = new SetPosition(Parts.hood, 0.9).requires(this);
    public Command reset = new SetPosition(Parts.hood, 0.0).requires(this);
}
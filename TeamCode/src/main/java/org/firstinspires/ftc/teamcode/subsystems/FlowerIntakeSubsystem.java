package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

/**
 * Flower intake: one servo that opens or closes on a command and stays there until the next one.
 * Nothing is sent until the first command, so it doesn't move during INIT.
 * The open / closed positions are in SubsystemConstants (Dashboard).
 */
public class FlowerIntakeSubsystem implements Subsystem {
    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private ServoImplEx flower;
    private boolean isInitialized = false;

    private boolean flowerCommanded = false;      // false until the first setOpen
    private boolean flowerOpen = false;
    private double lastFlowerPos = Double.NaN;    // what was last sent to the servo
    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;

    public FlowerIntakeSubsystem (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command defaultCommand){
        this.defaultCommand = defaultCommand;
    }

    @Override
    public void initialize(){
        flower = hardwareMap.get(ServoImplEx.class, "flower");

        flower.setPwmRange(AxonPWMRange);

        flower.setDirection(Servo.Direction.FORWARD);

        flowerCommanded = false;
        flowerOpen = false;
        lastFlowerPos = Double.NaN;

        isInitialized = true;
    }

    @Override
    public void periodic(){
        if (!isInitialized) return;

        updateFlower();
    }

    /**
     * Sends the servo position, only when it changes. The positions are read every loop,
     * so a Dashboard change moves the servo right away.
     */
    private void updateFlower() {
        if (!flowerCommanded) return;   // never asked: send nothing
        double pos = flowerOpen ? SubsystemConstants.FlowerOpenPos : SubsystemConstants.FlowerClosedPos;
        if (pos != lastFlowerPos) {
            flower.setPosition(pos);
            lastFlowerPos = pos;
        }
    }

    // ----- Abilities that commands can use -----

    /** true = open, false = closed. The servo holds it until the next command. */
    public void setOpen(boolean open) {
        flowerOpen = open;
        flowerCommanded = true;
    }

    /** Open if it's closed, closed if it's open. Never commanded counts as closed, so the first toggle opens. */
    public void toggle() {
        setOpen(!isOpen());
    }

    /** True once told to open (closed or never commanded = false). */
    public boolean isOpen() {
        return flowerCommanded && flowerOpen;
    }
}

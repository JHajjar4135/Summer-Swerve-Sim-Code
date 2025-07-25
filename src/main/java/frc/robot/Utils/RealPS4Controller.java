package frc.robot.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class RealPS4Controller extends CommandPS4Controller{
    private final double deadband;
    private final double debounce;

    public RealPS4Controller(int port, double deadband, double debounce){
        super(port);
        this.deadband = deadband;
        this.debounce = debounce;
    }

    @Override
    public double getLeftX(){
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    @Override
    public double getLeftY(){
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    @Override
    public double getRightX(){
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    @Override
    public double getRightY(){
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }

    @Override
    public double getL2Axis(){
        return MathUtil.applyDeadband(super.getL2Axis(), deadband);
    }

    @Override
    public double getR2Axis(){
        return MathUtil.applyDeadband(super.getR2Axis(), deadband);
    }

    @Override
    public Trigger square(){
        return super.square().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger triangle(){
        return super.triangle().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger circle(){
        return super.circle().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger cross(){
        return super.cross().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger L1(){
        return super.L1().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger R1(){
        return super.R1().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger povUp(){
        return super.povUp().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger povDown(){
        return super.povDown().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger povLeft(){
        return super.povLeft().debounce(debounce,Debouncer.DebounceType.kBoth);
    }

    @Override
    public Trigger povRight(){
        Trigger povRight = super.povRight().debounce(debounce,Debouncer.DebounceType.kBoth);
        return povRight;
    }
}

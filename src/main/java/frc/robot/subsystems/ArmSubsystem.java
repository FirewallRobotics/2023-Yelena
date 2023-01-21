package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends PIDSubsystem {
    private WPI_TalonSRX talonSRXext; 
    private WPI_TalonSRX talonSRXlift; 
    public ArmSubsystem() {
        super( new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
        new TalonSRX(ArmConstants.kArmExtendMotor); 
        new TalonSRX(ArmConstants.kArmLiftMotor); 
    }

    public void ArmExtendMedCommand() {
        talonSRXlift.set(ArmConstants.kMidPosition); 
        talonSRXext.set(ArmConstants.kMidLength);
        System.out.println("Extending to medium height...");
    
    }
    
    public void ArmExtendHighCommand() {
        talonSRXlift.set(ArmConstants.kHighPosition);
        talonSRXext.set(ArmConstants.kHighLength);
        System.out.println("Extending to maximum height...");
    }

    public void ArmDefaultPositionCommand() {
        talonSRXlift.set(ArmConstants.kDefaultPosition);
        System.out.println("Returning to default position...");
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }
    
}

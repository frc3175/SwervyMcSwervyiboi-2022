package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToTheta extends CommandBase {

    private double m_desiredAngle;
    private SwerveDrivetrain m_drivetrain;
    private XboxController m_controller;

    public TurnToTheta(SwerveDrivetrain drivetrain, XboxController controller, double angle) {
        m_desiredAngle = angle;
        m_drivetrain = drivetrain;
        m_controller = controller;

        addRequirements(m_drivetrain);
    }

    @Override 
    public void execute() {

        double currentAngle = m_drivetrain.getAngle();

        boolean atAngle = Math.abs(currentAngle - m_desiredAngle) < 1;

        if(!atAngle) {
            
        }

    }

}

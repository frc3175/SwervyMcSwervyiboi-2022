package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public AHRS m_gyro;

    public SwerveDrivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_gyro.reset();
        
        m_swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw());

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                             Constants.FRONT_LEFT_OFFSET, 
                             Constants.FRONT_LEFT_AZIMUTH, 
                             Constants.FRONT_LEFT_DRIVE, 
                             Constants.FRONT_LEFT_ENCODER, 
                             Constants.FRONT_LEFT_AZIMUTH_REVERSED,
                             Constants.FRONT_LEFT_DRIVE_REVERSED,
                             Constants.FRONT_LEFT_CANCODER_REVERSED),
            new SwerveModule(1,
                             Constants.FRONT_RIGHT_OFFSET,
                             Constants.FRONT_RIGHT_AZIMUTH,
                             Constants.FRONT_RIGHT_DRIVE,
                             Constants.FRONT_RIGHT_ENCODER,
                             Constants.FRONT_RIGHT_AZIMUTH_REVERSED,
                             Constants.FRONT_RIGHT_DRIVE_REVERSED,
                             Constants.FRONT_RIGHT_CANCODER_REVERSED),
            new SwerveModule(2,
                             Constants.BACK_LEFT_OFFSET,
                             Constants.BACK_LEFT_AZIMUTH,
                             Constants.BACK_LEFT_DRIVE,
                             Constants.BACK_LEFT_ENCODER,
                             Constants.BACK_LEFT_AZIMUTH_REVERSED,
                             Constants.BACK_LEFT_DRIVE_REVERSED,
                             Constants.BACK_LEFT_CANCODER_REVERSED),
            new SwerveModule(3,
                             Constants.BACK_RIGHT_OFFSET,
                             Constants.BACK_RIGHT_AZIMUTH,
                             Constants.BACK_RIGHT_DRIVE,
                             Constants.BACK_RIGHT_ENCODER,
                             Constants.BACK_RIGHT_AZIMUTH_REVERSED,
                             Constants.BACK_RIGHT_DRIVE_REVERSED,
                             Constants.BACK_RIGHT_CANCODER_REVERSED)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for(SwerveModule module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }


    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        
        for(SwerveModule mod : m_swerveModules){
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveModules){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public Rotation2d getYaw() {
        return (Constants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) 
                                             : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void resetGyro() {
        m_gyro.reset();
    }

    public void keepModulesWhereTheyAre() {

        for(SwerveModule mod : m_swerveModules) {
            mod.keepModuleWhereItIs(mod.m_moduleNumber);
        }

    }

    public void zeroModules() {

        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }

    }

    @Override
    public void periodic(){
        m_swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : m_swerveModules){
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Drive Encoder", mod.getDriveEncoder());    
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Azimuth angle", mod.getState().angle.getDegrees());
        }

        SmartDashboard.putNumber("Gyro Yaw: ", m_gyro.getYaw());

        SmartDashboard.putNumber("Test value", SmartDashboard.getNumber("Mod 1 Azimuth from Cancoder", 0));

    }

    
}


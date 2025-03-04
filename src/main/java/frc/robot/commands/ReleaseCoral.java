// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// // 
// package frc.robot.commands;

// import java.lang.module.ModuleDescriptor.Requires;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Gripper;
// import frc.robot.subsystems.ArmState;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ReleaseCoral extends Command {

//     ArmStateManager armStateManager ;
//     Arm armSub ;
//     Gripper gripper ;

//     /** Creates a new EjectCoral. */
//     public ReleaseCoral(Gripper g) {

//         gripper = g ;

//         // Use addRequirements() here to declare subsystem dependencies.
//         addRequirements( g) ;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         gripper.setReleaseMode();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         if ( armSub.getArmState() != ArmState.Level1 ) {
//             return true;
//         } else {
//             return false ;
//         }
//     }
// }

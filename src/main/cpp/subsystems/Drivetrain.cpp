// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "Constants.h"
#include <cmath>

Drivetrain::Drivetrain() {
    driveModuleFL = std::make_shared<DriveModule>(Constants::MotorIDs::driveFL, Constants::MotorIDs::turnFL, Constants::EncoderIDs::encoderFL, p, i, d, iZ);     
    driveModuleFR = std::make_shared<DriveModule>(Constants::MotorIDs::driveFR, Constants::MotorIDs::turnFR, Constants::EncoderIDs::encoderFR, p, i, d, iZ); 
    driveModuleRL = std::make_shared<DriveModule>(Constants::MotorIDs::driveRL, Constants::MotorIDs::turnRL, Constants::EncoderIDs::encoderRL, p, i, d, iZ); 
    driveModuleRR = std::make_shared<DriveModule>(Constants::MotorIDs::driveRR, Constants::MotorIDs::turnRR, Constants::EncoderIDs::encoderRR, p, i, d, iZ); 
}

void Drivetrain::Periodic() {
    // printf("FL position: %f\n", driveModuleFL->GetTurnEncPosition());
    // printf("FR position: %f\n", driveModuleFR->GetTurnEncPosition());
    // printf("RL position: %f\n", driveModuleRL->GetTurnEncPosition());
    // printf("RR position: %f\n", driveModuleRR->GetTurnEncPosition());
}

void Drivetrain::SetDriveSpeed(double fl, double fr, double rl, double rr) {
    driveModuleFL->SetDriveSpeed(fl);
    driveModuleFR->SetDriveSpeed(fr);
    driveModuleRL->SetDriveSpeed(rl);
    driveModuleRR->SetDriveSpeed(rr);
    //printf("fl: %f, fr: %f, rl: %f, rr, %f\n", fl, fr, rl, rr);
}

void Drivetrain::SetTurnSpeed(double fl, double fr, double rl, double rr) {
    driveModuleFL->SetTurnSpeed(fl);
    driveModuleFR->SetTurnSpeed(fr);
    driveModuleRL->SetTurnSpeed(rl);
    driveModuleRR->SetTurnSpeed(rr);
}

void Drivetrain::SetLocation(double fl, double fr, double rl, double rr) {
    driveModuleFL->SetTurnLocation(fl);
    driveModuleFR->SetTurnLocation(fr);
    driveModuleRL->SetTurnLocation(rl);
    driveModuleRR->SetTurnLocation(rr);
}

void Drivetrain::SetAllPosition(double loc) {
    driveModuleFL->SetTurnPosition(loc);
    driveModuleFR->SetTurnPosition(loc);
    driveModuleRL->SetTurnPosition(loc);
    driveModuleRR->SetTurnPosition(loc);
}

void Drivetrain::SetAllDriveSpeed(double speed) {
    SetDriveSpeed(speed, speed, speed, speed);
}

void Drivetrain::SetAllTurnSpeed(double speed) {
    SetTurnSpeed(speed, speed, speed, speed);
}

void Drivetrain::SetAllLocation(double loc) {
    SetLocation(loc, loc, loc, loc);
}

void Drivetrain::StopDrive() {
    driveModuleFL->StopDrive();
    driveModuleFR->StopDrive();
    driveModuleRL->StopDrive();
    driveModuleRR->StopDrive();
}

void Drivetrain::StopAll() {
    driveModuleFL->StopBoth();
    driveModuleFR->StopBoth();
    driveModuleRL->StopBoth();
    driveModuleRR->StopBoth();
}

void Drivetrain::SwerveDrive(double fwd, double str, double rot) {
    double a = str - (rot * (l / r));
    double b = str + (rot * (l / r));
    double c = fwd - (rot * (w / r));
    double d = fwd + (rot * (w / r));

    //printf("a: %f, b: %f, c: %f, d: %f\n", a, b, c, d);
    //printf("str: %f, fwd: %f, rot: %f\n", str, fwd, rot);

    double ws1 = sqrt((b * b) + (c * c));
    double ws2 = sqrt((b * b) + (d * d));
    double ws3 = sqrt((a * a) + (d * d));
    double ws4 = sqrt((a * a) + (c * c));

    double wa1 = atan2(b, c) * 180 / M_PI;
    double wa2 = atan2(b, d) * 180 / M_PI;
    double wa3 = atan2(a, d) * 180 / M_PI;
    double wa4 = atan2(a, c) * 180 / M_PI;

    printf("Angles: wa4: %f, wa2: %f, wa1: %f, wa3: %f\n", wa4, wa2, wa1, wa3);

    double max = ws1;
    max = std::max(max, ws2);
    max = std::max(max, ws3);
    max = std::max(max, ws4);
    if (max > 1) {
        ws1 /= max;
        ws2 /= max;
        ws3 /= max;
        ws4 /= max;
    }
    printf("SetDriveSpeed: ws4: %f, ws2: %f, ws1: %f, ws3: %f", ws4, ws2, ws1, ws3);
    SetDriveSpeed(ws4, ws2, ws1, ws3);
    SetLocation(
        GetAngleToLocation(wa4),
        GetAngleToLocation(wa2),
        GetAngleToLocation(wa1),
        GetAngleToLocation(wa3)
    );
}

void Drivetrain::HumanDrive(double fwd, double str, double rot) {
    if (abs(rot) < 0.01)
        rot = 0;

    if (abs(fwd) < .15 && abs(str) < .15 && abs(rot) < 0.01) {
        StopDrive();
    } else {
        SetDriveBrakeMode(false);
        SwerveDrive(fwd, str, rot);
    }
}

void Drivetrain::TankDrive(double left, double right) {
    SetAllLocation(0);
    SetDriveSpeed(right, left, right, left);
}

double Drivetrain::GetAngleToLocation(double ang) {
    if (ang < 0)
        return 0.5 + ((180 - abs(ang)) / 360);
    else
        return ang / 360;
}

void Drivetrain::SetDriveBrakeMode(bool brake) {
    if (brake) {
        driveModuleFL->SetIdleMode(DriveModule::IdleMode::Brake);
        driveModuleFR->SetIdleMode(DriveModule::IdleMode::Brake);
        driveModuleRL->SetIdleMode(DriveModule::IdleMode::Brake);
        driveModuleRR->SetIdleMode(DriveModule::IdleMode::Brake);
    } else {
        driveModuleFL->SetIdleMode(DriveModule::IdleMode::Coast);
        driveModuleFR->SetIdleMode(DriveModule::IdleMode::Coast);
        driveModuleRL->SetIdleMode(DriveModule::IdleMode::Coast);
        driveModuleRR->SetIdleMode(DriveModule::IdleMode::Coast);
    }
}
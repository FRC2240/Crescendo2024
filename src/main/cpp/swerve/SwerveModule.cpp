#include "swerve/SwerveModule.h"
#include "swerve/ngr.h"
#include "frc/smartdashboard/SmartDashboard.h"
// #include <numbers>
#include <units/angular_velocity.h>
#include <iostream>
#include <fmt/format.h>
#ifndef CFG_NO_DRIVEBASE
#define CAN_BUS_NAME "swervecan"

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// Set Variables
using namespace ctre::phoenix6;
SwerveModule::SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, units::angle::turn_t offset)
    : driver{driver_adr, CAN_BUS_NAME},
      turner{turner_adr, CAN_BUS_NAME},
      cancoder{cancoder_adr, CAN_BUS_NAME}
{

    fmt::println("Driver version: {}", driver.GetVersionMajor().GetValue());
    std::cout << "Driver Pro: " << driver.GetIsProLicensed().GetValue() << "\n";
    std::cout << "Turner Pro: " << turner.GetIsProLicensed().GetValue() << "\n";
    std::cout << "Cancoder Pro" << cancoder.GetIsProLicensed().GetValue() << "\n";
    std::cout << "Cancoder pro msg: " << cancoder.GetIsProLicensed().IsAllGood();
    std::cout << "Swerve constuctor start for " << driver.GetDescription() << std::endl;
    turner_addr = turner_adr;

    configs::CANcoderConfiguration cancoder_config{};
    cancoder_config.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
    cancoder_config.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
    cancoder_config.MagnetSensor.MagnetOffset = offset.value();

    cancoder.GetConfigurator().Apply(cancoder_config);

    // Configure Driver    ctre::phoenix6::configs::t driver_config{};
    configs::TalonFXConfiguration driver_config{};
    driver_config.Audio.BeepOnBoot = true;
    driver_config.Audio.BeepOnConfig = true;

    driver_config.Slot0.kP = 0.02;
    // driver_config.Slot0.kD = 0.002;
    // driver_config.Slot0.kI = 0.4;
    // driver_config.Slot0.kV = 0.0097;
    // driver_config.Slot0.kD = 0.002; // I is bad, don't use
    // driver_config.integralZone = 200;
    // driver_config.Slot0.kI = 0.400;
    // driver_config.Slot0.kV = 0.0097; // FIXME could be kG, kA or Kv
    driver_config.CurrentLimits.StatorCurrentLimitEnable = true;
    driver_config.CurrentLimits.StatorCurrentLimit = 700;
    // driver_config.CurrentLimits.SupplyCurrentLimitEnable
    driver_config.MotorOutput.NeutralMode.value = driver_config.MotorOutput.NeutralMode.Brake;
    //  TODO: TUNING
    driver.SetInverted(false);
    driver.GetConfigurator().Apply(driver_config);

    // Configure Turner

    ctre::phoenix6::configs::TalonFXConfiguration turner_config{};
    turner_config.Audio.BeepOnBoot = true;
    turner_config.Slot0.kP = -3.203;
    // turner_config.Slot0.kI = 32;
    // turner_config.Slot0.kD = 0.08;
    turner_config.Slot0.kS = 0;
    // turner_config.Slot0.
    turner_config.MotorOutput.PeakForwardDutyCycle = .5;
    turner_config.MotorOutput.PeakReverseDutyCycle = -.5;
    turner_config.ClosedLoopGeneral.ContinuousWrap = 1;
    // turner_neutralDeadband = 0.07;
    // turner_eakOutputForward = .5;
    // turner_config.peakOutputReverse = -.5;
    // turner_config.Feedback.FeedbackRemoteSensorID = cancoder.GetDeviceID();
    // turner_config.remoteFilter0.remoteSensorDeviceID = cancoder.GetDeviceNumber();
    // turner_config.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_CANCoder;
    // turner_config.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    // turner_config.closedloopRamp = .000;
    // turner.ConfigAllSettings(turner_config);
    turner.SetInverted(false);
    // std::cout << "Swerve constuctor end \n";
    turner_config.Feedback.WithRemoteCANcoder(cancoder);
    turner_config.Feedback.SensorToMechanismRatio = 1;
    turner_config.Feedback.RotorToSensorRatio = TURNER_GEAR_RATIO;
    turner_config.MotorOutput.NeutralMode = 1;
    // turner_config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    turner.GetConfigurator().Apply(turner_config);
}

frc::SwerveModuleState SwerveModule::getState()
{
    frc::SwerveModuleState ret;
    ret.speed = units::meters_per_second_t{wheel_speed_to_bot_speed(driver.GetVelocity().Refresh().GetValue())};
    ret.angle = frc::Rotation2d(getAngle());

    return ret;
}

frc::SwerveModulePosition SwerveModule::getPosition()
{
    frc::SwerveModulePosition ret;
    ret.distance = units::meter_t{driver.GetPosition().Refresh().GetValue().value() * WHEEL_CIRCUMFERENCE.value()};
    ret.angle = frc::Rotation2d(getAngle());
    return ret;
}

units::degree_t SwerveModule::getAngle() { return get_angle_degrees(); }

units::degree_t SwerveModule::get_angle_degrees()
{
    return units::degree_t{cancoder.GetAbsolutePosition().Refresh().GetValue()};
}

units::angle::turn_t SwerveModule::get_angle_turns() { return cancoder.GetAbsolutePosition().Refresh().GetValue(); }

double SwerveModule::getDriverTemp() { return driver.GetDeviceTemp().Refresh().GetValue().value(); }

double SwerveModule::getTurnerTemp() { return turner.GetDeviceTemp().Refresh().GetValue().value(); }

void SwerveModule::setDesiredState(frc::SwerveModuleState const &desired_state)
{
    frc::Rotation2d const current_rotation = get_angle_degrees();

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto const [optimized_speed, optimized_angle] = frc::SwerveModuleState::Optimize(desired_state, current_rotation);
    // auto const optimized_speed = desired_state.speed;
    // auto const optimized_angle = desired_state.angle;
    // Difference between desired angle and current angle
    // frc::Rotation2d delta_rotation = optimized_angle - current_rotation;

    controls::PositionDutyCycle controlreq{optimized_angle.Degrees()};
    controlreq.EnableFOC = 1;
    // Convert change in angle to change in (cancoder) ticks
    // double const delta_ticks = delta_rotation.Degrees().value() * TICKS_PER_CANCODER_DEGREE;
    frc::SmartDashboard::SmartDashboard::PutNumber(driver.GetDescription() + "/desired speed", optimized_speed.value());
    frc::SmartDashboard::SmartDashboard::PutNumber(driver.GetDescription() + "/desired angle", units::turn_t{optimized_angle.Degrees()}.value());
    driver.SetControl(controls::VelocityDutyCycle{bot_speed_to_wheel_speed(optimized_speed)});
    turner.SetControl(controlreq);
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/vout", driver.GetMotorVoltage().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/cyclemarker", (double)std::rand() / RAND_MAX);
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/turner.get()", turner.GetPosition().Refresh().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/driver rotations", driver.GetPosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/getAngle().value()", getAngle().value());
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/cancoder.value().value()", cancoder.GetAbsolutePosition().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(driver.GetDescription() + "/latency", cancoder.GetAbsolutePosition().GetTimestamp().GetLatency().convert<units::time::millisecond>().value());
    // turner.SetControl(controls::PositionVoltage(units::angle::turn_t{0.4}));
    // driver.SetControl(ctre::phoenix6::controls::PositionDutyCycle, desired_driver_velocity_ticks);

    // turner.Set(TalonFXControlMode::Position, desired_turner_pos_ticks);
}

inline units::turns_per_second_t SwerveModule::bot_speed_to_wheel_speed(units::meters_per_second_t bot_speed)
{
    return units::turns_per_second_t{(bot_speed.value() / WHEEL_CIRCUMFERENCE.value())};
}

inline units::meters_per_second_t SwerveModule::wheel_speed_to_bot_speed(units::turns_per_second_t wheel_speed)
{
    return units::meters_per_second_t{wheel_speed.value() * WHEEL_CIRCUMFERENCE.value()};
}

void SwerveModule::percentOutputControl(double const &percent_output)
{
    driver.SetControl(controls::DutyCycleOut{percent_output});
    turner.SetControl(controls::DutyCycleOut{0});
    // turner.Set();
}

void SwerveModule::manualVelocityContol(double const &velocity_ticks_per_100ms)
{
    // FIXME
    driver.SetControl(controls::DutyCycleOut{velocity_ticks_per_100ms});
    turner.SetControl(controls::DutyCycleOut{0});
}
#endif

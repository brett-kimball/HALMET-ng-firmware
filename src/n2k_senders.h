#ifndef HALMET_SRC_N2K_SENDERS_H_
#define HALMET_SRC_N2K_SENDERS_H_

#include <N2kMessages.h>
#include <NMEA2000.h>

#include "sensesp/system/saveable.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/repeat.h"
#include "sensesp_base_app.h"

namespace halmet {

// ========================================================================
// NMEA 2000 SENDERS — PGN TRANSMITTERS
// ========================================================================

/**
 * @brief Transmit NMEA 2000 PGN 127488: Engine Parameters, Rapid Update
 *
 * Sends high-frequency engine data: RPM, boost pressure, tilt/trim.
 */
class N2kEngineParameterRapidSender : public sensesp::FileSystemSaveable {
 public:
  N2kEngineParameterRapidSender(
      String config_path,
      uint8_t engine_instance,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // ms — NMEA 2000 standard
        expiry_{1000}           // ms — input timeout
  {
    this->initialize_members(repeat_interval_, expiry_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kEngineParamRapid(
          N2kMsg,
          this->engine_instance_,
          this->engine_speed_rpm_->get(),
          this->engine_boost_pressure_->get(),
          this->engine_tilt_trim_->get()
      );
      this->nmea2000_->SendMsg(N2kMsg);
    });

    engine_speed_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [](double value) { return 60 * value; })
    )->connect_to(engine_speed_rpm_);
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool from_json(const JsonObject& config) override {
    if (!config["engine_instance"].is<int>()) {
      return false;
    }
    engine_instance_ = config["engine_instance"];
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["engine_instance"] = engine_instance_;
    return true;
  }

  // --------------------------------------------------------------------
  // INPUTS
  // --------------------------------------------------------------------
  sensesp::ObservableValue<double> engine_speed_;  // Hz → RPM
  std::shared_ptr<sensesp::RepeatExpiring<double>> engine_boost_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<int8_t>> engine_tilt_trim_;

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  std::shared_ptr<sensesp::RepeatExpiring<double>> engine_speed_rpm_;

  uint8_t engine_instance_ = 0;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    engine_boost_pressure_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
    engine_tilt_trim_ = std::make_shared<sensesp::RepeatExpiring<int8_t>>(
        repeat_interval, expiry
    );
    engine_speed_rpm_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
  }
};

const String ConfigSchema(const N2kEngineParameterRapidSender& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "engine_instance": {
        "title": "Engine instance",
        "type": "integer",
        "description": "Engine NMEA 2000 instance number (0-253)"
      }
    }
  })###";
}

/**
 * @brief Transmit NMEA 2000 PGN 127489: Engine Parameters, Dynamic
 *
 * Sends detailed engine status: pressures, temperatures, alarms.
 */
class N2kEngineParameterDynamicSender : public sensesp::FileSystemSaveable {
 public:
  N2kEngineParameterDynamicSender(
      String config_path,
      uint8_t engine_instance,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        engine_instance_{engine_instance},
        nmea2000_{nmea2000},
        repeat_interval_{500},  // ms — NMEA 2000 standard
        expiry_{5000}           // ms — input timeout
  {
    this->initialize_members(repeat_interval_, expiry_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kEngineDynamicParam(
          N2kMsg,
          this->engine_instance_,
          this->oil_pressure_->get(),
          this->oil_temperature_->get(),
          this->temperature_->get(),
          this->alternator_potential_->get(),
          this->fuel_rate_->get(),
          this->total_engine_hours_->get(),
          this->coolant_pressure_->get(),
          this->fuel_pressure_->get(),
          this->engine_load_->get(),
          this->engine_torque_->get(),
          this->get_engine_status_1(),
          this->get_engine_status_2()
      );
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool from_json(const JsonObject& config) override {
    if (!config["engine_instance"].is<int>()) {
      return false;
    }
    engine_instance_ = config["engine_instance"];
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["engine_instance"] = engine_instance_;
    return true;
  }

  // --------------------------------------------------------------------
  // INPUTS — ANALOG VALUES
  // --------------------------------------------------------------------
  std::shared_ptr<sensesp::RepeatExpiring<double>> oil_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> oil_temperature_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> temperature_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> alternator_potential_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> fuel_rate_;
  std::shared_ptr<sensesp::RepeatExpiring<uint32_t>> total_engine_hours_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> coolant_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> fuel_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<int>> engine_load_;
  std::shared_ptr<sensesp::RepeatExpiring<int>> engine_torque_;

  // --------------------------------------------------------------------
  // INPUTS — DISCRETE ALARMS (STATUS 1)
  // --------------------------------------------------------------------
  std::shared_ptr<sensesp::RepeatExpiring<bool>> check_engine_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> over_temperature_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> low_oil_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> low_oil_level_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> low_fuel_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> low_system_voltage_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> low_coolant_level_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> water_flow_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> water_in_fuel_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> charge_indicator_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> preheat_indicator_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> high_boost_pressure_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> rev_limit_exceeded_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> egr_system_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> throttle_position_sensor_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> emergency_stop_;

  // --------------------------------------------------------------------
  // INPUTS — DISCRETE ALARMS (STATUS 2)
  // --------------------------------------------------------------------
  std::shared_ptr<sensesp::RepeatExpiring<bool>> warning_level_1_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> warning_level_2_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> power_reduction_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> maintenance_needed_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> engine_comm_error_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> sub_or_secondary_throttle_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> neutral_start_protect_;
  std::shared_ptr<sensesp::RepeatExpiring<bool>> engine_shutting_down_;

 protected:
  // --------------------------------------------------------------------
  // ENGINE STATUS 1 — BITFIELD CONSTRUCTION
  // --------------------------------------------------------------------
  tN2kEngineDiscreteStatus1 get_engine_status_1() {
    tN2kEngineDiscreteStatus1 status = 0;

    status.Bits.OverTemperature = over_temperature_->get();
    status.Bits.LowOilPressure = low_oil_pressure_->get();
    status.Bits.LowOilLevel = low_oil_level_->get();
    status.Bits.LowFuelPressure = low_fuel_pressure_->get();
    status.Bits.LowSystemVoltage = low_system_voltage_->get();
    status.Bits.LowCoolantLevel = low_coolant_level_->get();
    status.Bits.WaterFlow = water_flow_->get();
    status.Bits.WaterInFuel = water_in_fuel_->get();
    status.Bits.ChargeIndicator = charge_indicator_->get();
    status.Bits.PreheatIndicator = preheat_indicator_->get();
    status.Bits.HighBoostPressure = high_boost_pressure_->get();
    status.Bits.RevLimitExceeded = rev_limit_exceeded_->get();
    status.Bits.EGRSystem = egr_system_->get();
    status.Bits.ThrottlePositionSensor = throttle_position_sensor_->get();
    status.Bits.EngineEmergencyStopMode = emergency_stop_->get();

    // CheckEngine = any other alarm
    status.Bits.CheckEngine =
        status.Bits.OverTemperature || status.Bits.LowOilPressure ||
        status.Bits.LowOilLevel || status.Bits.LowFuelPressure ||
        status.Bits.LowSystemVoltage || status.Bits.LowCoolantLevel ||
        status.Bits.WaterFlow || status.Bits.WaterInFuel ||
        status.Bits.ChargeIndicator || status.Bits.PreheatIndicator ||
        status.Bits.HighBoostPressure || status.Bits.RevLimitExceeded ||
        status.Bits.EGRSystem || status.Bits.ThrottlePositionSensor ||
        status.Bits.EngineEmergencyStopMode;

    return status;
  }

  // --------------------------------------------------------------------
  // ENGINE STATUS 2 — BITFIELD CONSTRUCTION
  // --------------------------------------------------------------------
  tN2kEngineDiscreteStatus2 get_engine_status_2() {
    tN2kEngineDiscreteStatus2 status = 0;
    status.Bits.WarningLevel1 = warning_level_1_->get();
    status.Bits.WarningLevel2 = warning_level_2_->get();
    status.Bits.LowOiPowerReduction = power_reduction_->get();
    status.Bits.MaintenanceNeeded = maintenance_needed_->get();
    status.Bits.EngineCommError = engine_comm_error_->get();
    status.Bits.SubOrSecondaryThrottle = sub_or_secondary_throttle_->get();
    status.Bits.NeutralStartProtect = neutral_start_protect_->get();
    status.Bits.EngineShuttingDown = engine_shutting_down_->get();
    return status;
  }

  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t engine_instance_;

 private:
  void initialize_members(uint32_t repeat_interval_, uint32_t expiry_) {
    oil_pressure_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    oil_temperature_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    temperature_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    alternator_potential_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    fuel_rate_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    total_engine_hours_ = std::make_shared<sensesp::RepeatExpiring<uint32_t>>(
        repeat_interval_, expiry_
    );
    coolant_pressure_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    fuel_pressure_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval_, expiry_
    );
    engine_load_ = std::make_shared<sensesp::RepeatExpiring<int>>(
        repeat_interval_, expiry_
    );
    engine_torque_ = std::make_shared<sensesp::RepeatExpiring<int>>(
        repeat_interval_, expiry_
    );
    check_engine_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    over_temperature_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    low_oil_pressure_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    low_oil_level_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    low_fuel_pressure_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    low_system_voltage_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    low_coolant_level_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    water_flow_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    water_in_fuel_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    charge_indicator_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    preheat_indicator_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    high_boost_pressure_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    rev_limit_exceeded_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    egr_system_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    throttle_position_sensor_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    emergency_stop_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    warning_level_1_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    warning_level_2_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    power_reduction_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    maintenance_needed_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    engine_comm_error_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    sub_or_secondary_throttle_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    neutral_start_protect_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
    engine_shutting_down_ = std::make_shared<sensesp::RepeatExpiring<bool>>(
        repeat_interval_, expiry_
    );
  }
};

const String ConfigSchema(const N2kEngineParameterDynamicSender& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "engine_instance": {
        "title": "Engine instance",
        "type": "integer",
        "description": "Engine NMEA 2000 instance number (0-253)"
      }
    }
  })###";
}

/**
 * @brief Transmit NMEA 2000 PGN 127505: Fluid Level
 *
 * Reports tank level as percentage and capacity.
 */
class N2kFluidLevelSender : public sensesp::FileSystemSaveable {
 public:
  N2kFluidLevelSender(
      String config_path,
      uint8_t tank_instance,
      tN2kFluidType tank_type,
      double tank_capacity,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        tank_instance_{tank_instance},
        tank_type_{tank_type},
        tank_capacity_{tank_capacity},
        nmea2000_{nmea2000},
        repeat_interval_{2500},  // ms — NMEA 2000 standard
        expiry_{10000}           // ms — input timeout
  {
    tank_level_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [this](double value) { return 100 * value; })
    )->connect_to(&tank_level_percent_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kFluidLevel(
          N2kMsg,
          this->tank_instance_,
          this->tank_type_,
          this->tank_level_percent_.get(),
          this->tank_capacity_
      );
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool from_json(const JsonObject& config) override {
    String expected[] = {"tank_instance", "tank_type", "tank_capacity"};
    for (auto str : expected) {
      if (!config[str].is<int>()) {
        debugE("N2kFluidLevelSender: Missing configuration key %s", str.c_str());
        return false;
      }
    }
    tank_instance_ = config["tank_instance"];
    tank_type_ = config["tank_type"];
    tank_capacity_ = config["tank_capacity"];
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["tank_instance"] = tank_instance_;
    config["tank_type"] = tank_type_;
    config["tank_capacity"] = tank_capacity_;
    return true;
  }

  // --------------------------------------------------------------------
  // INPUTS
  // --------------------------------------------------------------------
  sensesp::ObservableValue<double> tank_level_;  // ratio (0.0–1.0)

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;

  uint8_t tank_instance_;
  tN2kFluidType tank_type_;
  double tank_capacity_;  // liters
  sensesp::RepeatExpiring<double> tank_level_percent_{repeat_interval_, expiry_};
};

const String ConfigSchema(const N2kFluidLevelSender& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "tank_instance": {
        "title": "Tank instance",
        "type": "integer",
        "description": "Tank NMEA 2000 instance number (0-13)"
      },
      "tank_type": {
        "title": "Tank type",
        "type": "integer",
        "description": "Tank type (0-13)"
      },
      "tank_capacity": {
        "title": "Tank capacity",
        "type": "number",
        "description": "Tank capacity (liters)"
      }
    }
  })###";
}

/**
 * @brief Transmit NMEA 2000 PGN 127245: Rudder
 *
 * Sends rudder angle in radians.
 */
class N2kRudderSender : public sensesp::FileSystemSaveable {
 public:
  N2kRudderSender(
      String config_path,
      uint8_t rudder_instance,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        rudder_instance_{rudder_instance},
        nmea2000_{nmea2000},
        repeat_interval_{100},
        expiry_{1000}
  {
    this->initialize_members(repeat_interval_, expiry_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg msg;
      SetN2kPGN127245(
          msg,
          rudder_angle_rad_->get(),
          rudder_instance_,
          N2kRDO_NoDirectionOrder,
          N2kDoubleNA
      );
      nmea2000_->SendMsg(msg);
    });
  }

  // --------------------------------------------------------------------
  // INPUTS
  // --------------------------------------------------------------------
  sensesp::ObservableValue<double> rudder_angle_deg_;

 protected:
  tNMEA2000* nmea2000_;
  uint8_t rudder_instance_;
  unsigned int repeat_interval_;
  unsigned int expiry_;

  std::shared_ptr<sensesp::RepeatExpiring<double>> rudder_angle_rad_;

 private:
  void initialize_members(unsigned int repeat, unsigned int exp) {
    rudder_angle_rad_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat, exp
    );

    rudder_angle_deg_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [](double deg) { return deg * DEG_TO_RAD; })
    )->connect_to(rudder_angle_rad_);
  }
};

const String ConfigSchema(const N2kRudderSender& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "rudder_instance": {
        "title": "Rudder instance",
        "type": "integer",
        "description": "Rudder NMEA 2000 instance (0-15)"
      }
    }
  })###";
}

/**
 * @brief Transmit NMEA 2000 PGN 130576: Trim Tab Position
 */
class N2kTrimTabSender : public sensesp::FileSystemSaveable {
 public:
  N2kTrimTabSender(String config_path, tNMEA2000* nmea2000)
      : sensesp::FileSystemSaveable{config_path},
        nmea2000_{nmea2000},
        repeat_interval_{500},
        expiry_{5000} {
    this->initialize_members(repeat_interval_, expiry_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg msg;
      double port_deg = trim_deg_port_exp_->get();  // Always get the value
      double stbd_deg = trim_deg_stbd_exp_->get();  // Always get the value
      SetN2kPGN130576(msg, port_deg * DEG_TO_RAD, stbd_deg * DEG_TO_RAD);
      nmea2000_->SendMsg(msg);
    });
  }

  sensesp::ObservableValue<double> trim_deg_port_;
  sensesp::ObservableValue<double> trim_deg_stbd_;

 protected:
  tNMEA2000* nmea2000_;
  unsigned int repeat_interval_;
  unsigned int expiry_;

  std::shared_ptr<sensesp::RepeatExpiring<double>> trim_deg_port_exp_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> trim_deg_stbd_exp_;

 private:
  void initialize_members(unsigned int repeat, unsigned int exp) {
    trim_deg_port_exp_ = std::make_shared<sensesp::RepeatExpiring<double>>(repeat, exp);
    trim_deg_port_.connect_to(trim_deg_port_exp_);
    trim_deg_stbd_exp_ = std::make_shared<sensesp::RepeatExpiring<double>>(repeat, exp);
    trim_deg_stbd_.connect_to(trim_deg_stbd_exp_);
  }
};

const String ConfigSchema(const N2kTrimTabSender& obj) {
  return R"###({
    "type": "object",
    "properties": {}
  })###";
}

/**
 * @brief Transmit NMEA 2000 PGN 127493: Transmission Parameters, Dynamic
 *
 * Sends gear position (0=R, 1=N, 2=F, 3=Unknown), oil pressure, temperature.
 */
class N2kTransmissionSender : public sensesp::FileSystemSaveable {
 public:
  N2kTransmissionSender(
      String config_path,
      uint8_t transmission_instance,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        transmission_instance_{transmission_instance},
        nmea2000_{nmea2000},
        repeat_interval_{500},
        expiry_{5000}
  {
    this->initialize_members(repeat_interval_, expiry_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg msg;
      SetN2kPGN127493(
          msg,
          transmission_instance_,
          static_cast<tN2kTransmissionGear>(
              gear_code_->get().is_valid() ? gear_code_->get().value() : 3
          ),
          oil_pressure_pa_->get(),
          oil_temp_k_->get(),
          discrete_status1_->get()
      );
      nmea2000_->SendMsg(msg);
    });
  }

  // --------------------------------------------------------------------
  // INPUTS — PUBLIC
  // --------------------------------------------------------------------
  sensesp::ObservableValue<int> gear_;  // 0=Reverse, 1=Neutral, 2=Forward, 3=Unknown
  std::shared_ptr<sensesp::RepeatExpiring<double>> oil_pressure_pa_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> oil_temp_k_;
  std::shared_ptr<sensesp::RepeatExpiring<unsigned char>> discrete_status1_;

 protected:
  tNMEA2000* nmea2000_;
  uint8_t transmission_instance_;
  unsigned int repeat_interval_;
  unsigned int expiry_;

  std::shared_ptr<sensesp::RepeatExpiring<int>> gear_code_;

 private:
  void initialize_members(unsigned int repeat, unsigned int exp) {
    gear_code_ = std::make_shared<sensesp::RepeatExpiring<int>>(repeat, exp);
    oil_pressure_pa_ = std::make_shared<sensesp::RepeatExpiring<double>>(repeat, exp);
    oil_temp_k_ = std::make_shared<sensesp::RepeatExpiring<double>>(repeat, exp);
    discrete_status1_ = std::make_shared<sensesp::RepeatExpiring<unsigned char>>(repeat, exp);

    gear_.connect_to(gear_code_);
  }
};

const String ConfigSchema(const N2kTransmissionSender& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "transmission_instance": {
        "title": "Transmission instance",
        "type": "integer",
        "description": "Transmission NMEA 2000 instance (0-15)"
      }
    }
  })###";
};

/**
 * @brief Transmit NMEA 2000 PGN 127250: Vessel Heading
 *
 * Sends magnetic heading.
 */
class N2kHeadingSender : public sensesp::FileSystemSaveable {
 public:
  N2kHeadingSender(
      String config_path,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // ms — NMEA 2000 standard
        expiry_{1000}           // ms — input timeout
  {
    this->initialize_members(repeat_interval_, expiry_);

    heading_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [](double deg) { return deg * DEG_TO_RAD; })  // Convert to radians
    )->connect_to(heading_rad_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kMagneticHeading(
          N2kMsg,
          this->sid_++,
          this->heading_rad_->get(),
          0.0,  // Deviation (not available)
          0.0   // Variation (not available)
      );
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool from_json(const JsonObject& config) override {
    return true;  // No config needed
  }

  virtual bool to_json(JsonObject& config) override {
    return true;
  }

  // --------------------------------------------------------------------
  // INPUTS
  // --------------------------------------------------------------------
  sensesp::ObservableValue<double> heading_;  // Degrees

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;
  uint8_t sid_ = 0;

  std::shared_ptr<sensesp::RepeatExpiring<double>> heading_rad_;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    heading_rad_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
  }
};

const String ConfigSchema(const N2kHeadingSender& obj) {
  return R"###({
    "type": "object",
    "properties": {}
  })###";
};

/**
 * @brief Transmit NMEA 2000 PGN 127257: Attitude
 *
 * Sends pitch and roll angles.
 */
class N2kAttitudeSender : public sensesp::FileSystemSaveable {
 public:
  N2kAttitudeSender(
      String config_path,
      tNMEA2000* nmea2000
  )
      : sensesp::FileSystemSaveable{config_path},
        nmea2000_{nmea2000},
        repeat_interval_{100},  // ms — NMEA 2000 standard
        expiry_{1000}           // ms — input timeout
  {
    this->initialize_members(repeat_interval_, expiry_);

    pitch_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [](double deg) { return deg * DEG_TO_RAD; })  // Convert to radians
    )->connect_to(pitch_rad_);

    roll_.connect_to(
        new sensesp::LambdaTransform<double, double>(
            [](double deg) { return deg * DEG_TO_RAD; })  // Convert to radians
    )->connect_to(roll_rad_);

    sensesp::event_loop()->onRepeat(repeat_interval_, [this]() {
      tN2kMsg N2kMsg;
      SetN2kAttitude(
          N2kMsg,
          this->sid_++,
          this->roll_rad_->get(),
          this->pitch_rad_->get(),
          N2kDoubleNA  // Yaw not used, heading is separate
      );
      this->nmea2000_->SendMsg(N2kMsg);
    });
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool from_json(const JsonObject& config) override {
    return true;  // No config needed
  }

  virtual bool to_json(JsonObject& config) override {
    return true;
  }

  // --------------------------------------------------------------------
  // INPUTS
  // --------------------------------------------------------------------
  sensesp::ObservableValue<double> pitch_;  // Degrees
  sensesp::ObservableValue<double> roll_;   // Degrees
  sensesp::ObservableValue<double> yaw_;    // Degrees (optional, can be N/A)

 protected:
  unsigned int repeat_interval_;
  unsigned int expiry_;
  tNMEA2000* nmea2000_;
  uint8_t sid_ = 0;

  std::shared_ptr<sensesp::RepeatExpiring<double>> pitch_rad_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> roll_rad_;
  std::shared_ptr<sensesp::RepeatExpiring<double>> yaw_rad_;

 private:
  void initialize_members(unsigned int repeat_interval, unsigned int expiry) {
    pitch_rad_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
    roll_rad_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
    yaw_rad_ = std::make_shared<sensesp::RepeatExpiring<double>>(
        repeat_interval, expiry
    );
  }
};

const String ConfigSchema(const N2kAttitudeSender& obj) {
  return R"###({
    "type": "object",
    "properties": {}
  })###";
};

}  // namespace halmet

#endif  // HALMET_SRC_N2K_SENDERS_H_
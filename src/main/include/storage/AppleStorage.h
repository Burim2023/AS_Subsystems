#pragma once

#include <string>
#include <unordered_map>
#include <optional>
#include <chrono>
#include <stdexcept>

namespace storage {

/**
 * @brief Storage slot enumeration
 */
enum class Slot { S0 = 0, S1 = 1, S2 = 2 };

/**
 * @brief Configuration for apple storage positioning and slot selection
 */
struct StorageConfig {
    double t1;          ///< Threshold 1 for slot selection (maxtime < t1 → slot 0)
    double t2;          ///< Threshold 2 for slot selection (t1 ≤ maxtime < t2 → slot 1, else slot 2)
    double slot0Frac;   ///< Fraction of MaxTimeFrontToBack for slot 0 center (0,1]
    double slot1Frac;   ///< Fraction of MaxTimeFrontToBack for slot 1 center (0,1]
    double slot2Frac;   ///< Fraction of MaxTimeFrontToBack for slot 2 center (0,1]
    
    StorageConfig() : t1(1.0), t2(2.0), slot0Frac(0.17), slot1Frac(0.50), slot2Frac(0.83) {}
};

/**
 * @brief Exception thrown when storage is full
 */
class StorageFullError : public std::runtime_error {
public:
    StorageFullError() : std::runtime_error("All storage slots are occupied") {}
};

/**
 * @brief Exception thrown for invalid color values
 */
class InvalidColorError : public std::runtime_error {
public:
    InvalidColorError(const std::string& color) 
        : std::runtime_error("Invalid color '" + color + "'. Allowed: red, green, yellow") {}
};

/**
 * @brief Exception thrown for invalid slot numbers
 */
class InvalidSlotError : public std::runtime_error {
public:
    InvalidSlotError(int slot) 
        : std::runtime_error("Invalid slot " + std::to_string(slot) + ". Must be 0, 1, or 2") {}
};

/**
 * @brief Exception thrown for invalid time values
 */
class InvalidTimeError : public std::runtime_error {
public:
    InvalidTimeError() : std::runtime_error("Invalid time value (NaN or Inf)") {}
};

/**
 * @brief Framework-agnostic utility for apple storage mapping and persistence
 */
class AppleStorage {
public:
    /**
     * @brief Map maxtime to slot number using thresholds
     * @param maxtime The time value to map
     * @param cfg Configuration with t1, t2 thresholds
     * @return Slot number (0, 1, or 2)
     * @throws InvalidTimeError if maxtime is NaN or Inf
     */
    static int slot_from_maxtime(double maxtime, const StorageConfig& cfg);
    
    /**
     * @brief Calculate target time for slot center positioning
     * @param slot Target slot (0, 1, or 2)
     * @param maxTimeFrontToBack Calibrated max travel time
     * @param cfg Configuration with slot fractions
     * @return Target time in milliseconds
     */
    static std::chrono::milliseconds slot_center_time_ms(int slot, double maxTimeFrontToBack, const StorageConfig& cfg);
    
    /**
     * @brief Store an apple with fallback slot selection
     * @param color Apple color ("red", "green", "yellow")
     * @param maxtime Time value for slot selection
     * @param maxTimeFrontToBack Calibrated travel time
     * @param cfg Storage configuration
     * @return Chosen slot number
     * @throws StorageFullError if all slots occupied
     * @throws InvalidColorError if color invalid
     * @throws InvalidTimeError if maxtime invalid
     */
    int store(const std::string& color, double maxtime, double maxTimeFrontToBack, const StorageConfig& cfg);
    
    /**
     * @brief Get color stored in slot
     * @param slot Slot number (0, 1, or 2)
     * @return Color if slot occupied, std::nullopt if empty
     */
    std::optional<std::string> get_color(int slot) const;
    
    /**
     * @brief Remove apple from slot
     * @param slot Slot number (0, 1, or 2)
     * @return Color that was removed, std::nullopt if slot was empty
     */
    std::optional<std::string> remove(int slot);
    
    /**
     * @brief Check if all slots are occupied
     */
    bool is_full() const;
    
    /**
     * @brief Check if all slots are empty
     */
    bool is_empty() const;
    
    /**
     * @brief Serialize storage state to JSON
     * @return JSON string representation
     */
    std::string to_json() const;
    
    /**
     * @brief Deserialize storage state from JSON
     * @param json JSON string to parse
     */
    void from_json(std::string_view json);

private:
    std::unordered_map<int, std::string> slot_to_color; ///< Storage state map
    
    void validate_slot(int slot) const;
    void validate_color(const std::string& color) const;
};

} // namespace storage
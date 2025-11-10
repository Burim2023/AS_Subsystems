#include "storage/AppleStorage.h"
#include <cmath>
#include <sstream>
#include <set>
#include <cassert>

namespace storage {

int AppleStorage::slot_from_maxtime(double maxtime, const StorageConfig& cfg) {
    if (!std::isfinite(maxtime)) {
        throw InvalidTimeError();
    }
    
    if (maxtime < cfg.t1) {
        return 0;
    } else if (maxtime < cfg.t2) {
        return 1;
    } else {
        return 2;
    }
}

std::chrono::milliseconds AppleStorage::slot_center_time_ms(int slot, double maxTimeFrontToBack, const StorageConfig& cfg) {
    if (slot < 0 || slot > 2) {
        throw InvalidSlotError(slot);
    }
    
    double fraction;
    switch (slot) {
        case 0: fraction = cfg.slot0Frac; break;
        case 1: fraction = cfg.slot1Frac; break;
        case 2: fraction = cfg.slot2Frac; break;
        default: throw InvalidSlotError(slot);
    }
    
    double timeMs = fraction * maxTimeFrontToBack * 1000.0;
    return std::chrono::milliseconds(static_cast<long>(std::round(timeMs)));
}

int AppleStorage::store(const std::string& color, double maxtime, double maxTimeFrontToBack, const StorageConfig& cfg) {
    validate_color(color);
    
    // Get primary slot from maxtime
    int primarySlot = slot_from_maxtime(maxtime, cfg);
    
    // Try primary slot first, then fallback with wrap-around
    for (int attempt = 0; attempt < 3; ++attempt) {
        int candidateSlot = (primarySlot + attempt) % 3;
        
        if (slot_to_color.find(candidateSlot) == slot_to_color.end()) {
            // Slot is free
            slot_to_color[candidateSlot] = color;
            return candidateSlot;
        }
    }
    
    // All slots occupied
    throw StorageFullError();
}

std::optional<std::string> AppleStorage::get_color(int slot) const {
    validate_slot(slot);
    
    auto it = slot_to_color.find(slot);
    if (it != slot_to_color.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::optional<std::string> AppleStorage::remove(int slot) {
    validate_slot(slot);
    
    auto it = slot_to_color.find(slot);
    if (it != slot_to_color.end()) {
        std::string color = it->second;
        slot_to_color.erase(it);
        return color;
    }
    return std::nullopt;
}

bool AppleStorage::is_full() const {
    return slot_to_color.size() == 3;
}

bool AppleStorage::is_empty() const {
    return slot_to_color.empty();
}

std::string AppleStorage::to_json() const {
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (int slot = 0; slot < 3; ++slot) {
        if (!first) oss << ",";
        oss << "\"" << slot << "\":";
        
        auto color = get_color(slot);
        if (color) {
            oss << "\"" << *color << "\"";
        } else {
            oss << "null";
        }
        first = false;
    }
    oss << "}";
    return oss.str();
}

void AppleStorage::from_json(std::string_view json) {
    slot_to_color.clear();
    
    // Simple manual JSON parsing for {"0":"red","1":null,"2":"green"} format
    std::string str(json);
    
    for (int slot = 0; slot < 3; ++slot) {
        std::string slotKey = "\"" + std::to_string(slot) + "\":";
        size_t pos = str.find(slotKey);
        
        if (pos != std::string::npos) {
            size_t valueStart = pos + slotKey.length();
            
            if (str.substr(valueStart, 4) == "null") {
                // Slot is empty
                continue;
            } else if (str[valueStart] == '"') {
                // Find closing quote
                size_t valueEnd = str.find('"', valueStart + 1);
                if (valueEnd != std::string::npos) {
                    std::string color = str.substr(valueStart + 1, valueEnd - valueStart - 1);
                    slot_to_color[slot] = color;
                }
            }
        }
    }
}

void AppleStorage::validate_slot(int slot) const {
    if (slot < 0 || slot > 2) {
        throw InvalidSlotError(slot);
    }
}

void AppleStorage::validate_color(const std::string& color) const {
    static const std::set<std::string> validColors = {"red", "green", "yellow"};
    if (validColors.find(color) == validColors.end()) {
        throw InvalidColorError(color);
    }
}

#ifdef APPLE_STORAGE_SELFTEST
void run_self_tests() {
    StorageConfig cfg;
    cfg.t1 = 1.5;
    cfg.t2 = 3.0;
    
    // Test boundary behavior
    assert(AppleStorage::slot_from_maxtime(1.0, cfg) == 0);   // < t1
    assert(AppleStorage::slot_from_maxtime(1.5, cfg) == 1);   // == t1
    assert(AppleStorage::slot_from_maxtime(2.0, cfg) == 1);   // t1 < x < t2
    assert(AppleStorage::slot_from_maxtime(3.0, cfg) == 2);   // == t2
    assert(AppleStorage::slot_from_maxtime(4.0, cfg) == 2);   // > t2
    
    // Test invalid time
    try {
        AppleStorage::slot_from_maxtime(std::numeric_limits<double>::quiet_NaN(), cfg);
        assert(false); // Should throw
    } catch (const InvalidTimeError&) {
        // Expected
    }
    
    // Test slot center timing
    double maxTime = 10.0; // 10 second max travel
    auto slot0Time = AppleStorage::slot_center_time_ms(0, maxTime, cfg);
    auto slot1Time = AppleStorage::slot_center_time_ms(1, maxTime, cfg);
    auto slot2Time = AppleStorage::slot_center_time_ms(2, maxTime, cfg);
    
    assert(slot0Time.count() == static_cast<long>(std::round(0.17 * 10000))); // 1700ms
    assert(slot1Time.count() == static_cast<long>(std::round(0.50 * 10000))); // 5000ms
    assert(slot2Time.count() == static_cast<long>(std::round(0.83 * 10000))); // 8300ms
    
    // Test storage operations
    AppleStorage storage;
    assert(storage.is_empty());
    assert(!storage.is_full());
    
    // Store apples with fallback
    int slot0 = storage.store("red", 1.0, maxTime, cfg);    // Should go to slot 0
    int slot1 = storage.store("green", 2.0, maxTime, cfg);  // Should go to slot 1
    int slot2 = storage.store("yellow", 4.0, maxTime, cfg); // Should go to slot 2
    
    assert(slot0 == 0);
    assert(slot1 == 1);
    assert(slot2 == 2);
    assert(storage.is_full());
    
    // Test occupied slot fallback
    int fallbackSlot = storage.store("red", 1.0, maxTime, cfg); // Slot 0 occupied, should wrap to 1, but 1 occupied, then 2, but 2 occupied
    // Should throw StorageFullError
    
    // Test removal
    auto removedColor = storage.remove(1);
    assert(removedColor == "green");
    assert(!storage.is_full());
    
    // Now fallback should work
    int newSlot = storage.store("red", 1.0, maxTime, cfg); // Should fallback to slot 1
    assert(newSlot == 1);
    
    // Test JSON serialization
    std::string json = storage.to_json();
    AppleStorage storage2;
    storage2.from_json(json);
    
    assert(storage2.get_color(0) == "red");
    assert(storage2.get_color(1) == "red");
    assert(storage2.get_color(2) == "yellow");
}
#endif

} // namespace storage
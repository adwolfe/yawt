#ifndef TRACKEDITEMDATA_H
#define TRACKEDITEMDATA_H

#include <QPointF>
#include <QRectF>
#include <QString> // For typeToString and stringToType

// Enum for the type of tracked item
enum class ItemType {
    Worm,
    StartPoint,
    EndPoint,
    ControlPoint,
    Undefined // Default or unassigned
};

// Helper functions to convert ItemType to/from QString for display and editing
inline QString itemTypeToString(ItemType type) {
    switch (type) {
    case ItemType::Worm: return "Worm";
    case ItemType::StartPoint: return "Start Point";
    case ItemType::EndPoint: return "End Point";
    case ItemType::ControlPoint: return "Control Point";
    case ItemType::Undefined: return "Undefined";
    default: return "Unknown";
    }
}

inline ItemType stringToItemType(const QString& typeStr) {
    if (typeStr == "Worm") return ItemType::Worm;
    if (typeStr == "Start Point") return ItemType::StartPoint;
    if (typeStr == "End Point") return ItemType::EndPoint;
    if (typeStr == "Control Point") return ItemType::ControlPoint;
    return ItemType::Undefined;
}

// Structure to hold data for each item in the table
struct TrackedItem {
    int id;                         // Unique auto-generated ID
    ItemType type;                  // Type of the item
    QPointF initialCentroid;        // Centroid in video coordinates at selection
    QRectF initialBoundingBox;      // Bounding box in video coordinates at selection
    int frameOfSelection;           // Frame number where this item was selected
    // Add other relevant data as needed, e.g., color for display
};

#endif // TRACKEDITEMDATA_H

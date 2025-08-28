/*
 * Small translation unit to ensure the Qt Meta-Object Compiler (moc)
 * processes `CacheStatusWidget` and that the generated meta-object
 * code (vtable, staticMetaObject, signals/slots metadata) is linked.
 *
 * Some build configurations (or subtle differences between qmake/CMake versions)
 * may not generate/compile the moc output for a QObject-derived class unless
 * there is a dedicated .cpp that includes the header. Including the header here
 * provides a clear translation unit for automoc to pick up.
 */

#include "cachestatuswidget.h"

// Intentionally empty - the presence of this file (including the header)
// ensures automoc/moc will run and generate the necessary definitions.
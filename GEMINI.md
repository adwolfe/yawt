## Project Summary: YAWT (Yet Another Worm Tracker)

**Purpose:**
YAWT is a C++ desktop application designed for scientific image analysis, specifically for tracking the movement of *C. elegans* worms in video recordings of chemotaxis experiments.

**Core Technologies:**
- **C++17:** The primary programming language.
- **Qt6:** Used for the entire graphical user interface (GUI), including widgets, signals/slots, and threading.
- **OpenCV:** Used for all computer vision tasks, such as video reading, image processing (thresholding, blurring), and object detection.
- **CMake:** The build system used to configure and build the project.

**Architecture:**
The application is well-structured and follows a clear **Model-View-Controller (MVC)** design pattern, ensuring a strong separation between the UI and the core logic.

*   **View (`src/gui`):** The UI is managed by `MainWindow`, which acts as the main application shell. It owns all visual components (like the `VideoLoader` widget) but does not contain any business logic. It sends user commands to the `AppController` and updates itself based on signals received from the controller.

*   **Controller (`src/core`):** The `AppController` class is the heart of the application. It acts as the central coordinator, owning and managing the core non-UI components:
    *   `TrackingManager`: Executes the heavy-lifting of the worm tracking algorithms in a separate worker thread.
    *   `TrackingDataStorage`: An in-memory database holding all the application's state (worm data, tracks, etc.).
    *   Data Models: Qt-based models (`BlobTableModel`, `AnnotationTableModel`) that act as adapters between the raw data in `TrackingDataStorage` and the UI views.
    The `AppController` exposes a clean API for the `MainWindow` to call and forwards events (like tracking progress) from the `TrackingManager` to the UI via Qt signals.

*   **Model (`src/data`, `src/models`):** The data layer is composed of `TrackingDataStorage`, which holds the ground-truth data, and the Qt models in `src/models`, which expose that data to `QTableView` and other views in a format they can understand.

**Data Flow & Threading:**
1.  A user action in the `MainWindow` (e.g., drawing a Region of Interest) triggers a slot.
2.  The slot calls a method on the `AppController` (e.g., `appController->addRoi(...)`).
3.  For long-running tasks like tracking, the `AppController` delegates the work to the `TrackingManager`, which runs in a background thread to keep the UI responsive.
4.  The `TrackingManager` emits progress signals, which are caught by the `AppController` and forwarded to the `MainWindow` to update progress bars.
5.  When the task is complete, the `TrackingManager` emits a `finished` signal with the results.
6.  The `AppController` receives the results, updates the `TrackingDataStorage`, and emits a `tracksUpdated` signal.
7.  The `MainWindow` (and its models) receives this signal, and the UI automatically updates to display the new data. This entire cross-thread communication is handled safely by Qt's signal and slot mechanism.

#include "referenceannotationpanel.h"
#include "videoloader.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QToolButton>
#include <QScrollArea>
#include <QShortcut>
#include <QSpinBox>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QUndoCommand>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

using namespace ReferenceAnnotations;
namespace {
class Edit : public QUndoCommand {
public:
    Edit(Document before, Document after, std::function<void(const Document&)> apply, const QString& label)
        : QUndoCommand(label), m_before(std::move(before)), m_after(std::move(after)), m_apply(std::move(apply)) {}
    void undo() override { m_apply(m_before); }
    void redo() override { m_apply(m_after); }
private:
    Document m_before, m_after;
    std::function<void(const Document&)> m_apply;
};

QComboBox* combo(QWidget* parent, const QString& name, const QStringList& labels, const QStringList& values) {
    auto* c = new QComboBox(parent);
    c->setObjectName(name);
    for (int i = 0; i < labels.size(); ++i) c->addItem(labels[i], values[i]);
    return c;
}
} // namespace

ReferenceAnnotationPanel::ReferenceAnnotationPanel(VideoLoader* video, QWidget* parent)
    : QWidget(parent), m_video(video), m_undo(this) {
    setObjectName("referenceAnnotationPanel");
    m_undo.setUndoLimit(100);
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    auto* content = new QWidget(scroll);
    scroll->setWidget(content);
    outer->addWidget(scroll);
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(4);
    auto* tools = new QHBoxLayout;
    m_worm = new QSpinBox(this);
    m_worm->setObjectName("referenceWormId");
    m_worm->setRange(1, std::numeric_limits<int>::max());
    m_worm->setFixedWidth(80);
    m_worm->setToolTip("A persistent reference ID; it does not have to exist in the tracking results.");
    m_tool = new QComboBox(this);
    m_tool->setObjectName("referenceTool");
    m_tool->addItems({"Pan / zoom (Esc)", "Position (P)", "End A (A)", "End B (B)", "Centerline (C)"});
    m_tool->setToolTip("Position is an approximate center. For centerlines, click from A toward B; drag vertices to adjust; right-click a vertex to remove it.");
    tools->addWidget(new QLabel("Worm ID", this));
    tools->addWidget(m_worm);
    m_frameLabel = new QLabel("Load a video", this);
    tools->addWidget(m_frameLabel, 1, Qt::AlignRight);
    layout->addLayout(tools);
    layout->addWidget(m_tool);
    auto button = [this](QHBoxLayout* row, const QString& text, const QString& name, std::function<void()> action) {
        auto* b = new QToolButton(this);
        b->setText(text);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        b->setObjectName(name);
        row->addWidget(b);
        connect(b, &QToolButton::clicked, this, std::move(action));
        return b;
    };
    auto* geometry = new QHBoxLayout;
    button(geometry, "Clear tool", "referenceClearTool", [this] { clearTool(); });
    button(geometry, "Swap A/B", "referenceSwapEnds", [this] { swapEnds(); });
    auto* remove = button(geometry, "Delete", "referenceDeleteFrame", [this] { deleteFrame(); });
    remove->setToolTip("Delete this worm's annotation on the current frame. Undo restores it.");
    layout->addLayout(geometry);

    auto* labels = new QHBoxLayout;
    m_orientation = combo(this, "referenceOrientation", {"Head unknown", "A is head", "B is head"}, {"unknown", "a_is_head", "b_is_head"});
    m_visibility = combo(this, "referenceVisibility", {"Visible", "Partly visible", "Hidden"}, {"visible", "partial", "hidden"});
    m_event = combo(this, "referenceEvent", {"No event", "Self-contact", "Merge", "Split", "Lost", "Reversal"}, {"none", "self_contact", "merge", "split", "lost", "reversal"});
    m_review = combo(this, "referenceReview", {"Unreviewed", "Verified", "Uncertain"}, {"unreviewed", "verified", "uncertain"});
    m_review->setToolTip("Only explicitly verified annotations should be used as trusted accuracy references. Editing a verified annotation resets it to Unreviewed.");
    labels->addWidget(m_orientation, 1);
    labels->addWidget(m_visibility, 1);
    auto* eventLabels = new QHBoxLayout;
    eventLabels->addWidget(m_event, 1);
    eventLabels->addWidget(m_review, 1);
    for (auto* c : {m_orientation, m_visibility, m_event, m_review}) {
        connect(c, &QComboBox::currentIndexChanged, this, [this] { changeLabel(); });
    }
    layout->addLayout(labels);
    layout->addLayout(eventLabels);
    m_notes = new QLineEdit(this);
    m_notes->setObjectName("referenceNotes");
    m_notes->setPlaceholderText("Notes for this worm and frame; leave invisible endpoints or body routes unmarked");
    layout->addWidget(m_notes);
    // Commit text as it is entered so a seek from the existing player cannot
    // accidentally attach notes to the following frame.
    connect(m_notes, &QLineEdit::textEdited, this, [this] { changeLabel(); });

    auto* navigation = new QHBoxLayout;
    button(navigation, "‹ Frame", "referencePreviousFrame", [this] { navigate(-1, false); });
    button(navigation, "Frame ›", "referenceNextFrame", [this] { navigate(1, false); });
    m_step = new QSpinBox(this);
    m_step->setObjectName("referenceFrameStep");
    m_step->setRange(1, 1000000);
    m_step->setFixedWidth(80);
    m_step->setPrefix("Step ");
    navigation->addWidget(m_step);
    layout->addLayout(navigation);
    auto* marked = new QHBoxLayout;
    button(marked, "‹ Mark", "referencePreviousMarked", [this] { navigate(-1, true); })->setToolTip("Previous annotated frame for this worm (Shift + comma)");
    button(marked, "Mark ›", "referenceNextMarked", [this] { navigate(1, true); })->setToolTip("Next annotated frame for this worm (Shift + period)");
    auto* undo = button(marked, "Undo", "referenceUndo", [this] { finishDrag(); m_undo.undo(); });
    auto* redo = button(marked, "Redo", "referenceRedo", [this] { finishDrag(); m_undo.redo(); });
    undo->setEnabled(false);
    redo->setEnabled(false);
    connect(&m_undo, &QUndoStack::canUndoChanged, undo, &QToolButton::setEnabled);
    connect(&m_undo, &QUndoStack::canRedoChanged, redo, &QToolButton::setEnabled);
    layout->addLayout(marked);

    m_table = new QTableWidget(0, 5, this);
    m_table->setObjectName("referenceFrames");
    m_table->setHorizontalHeaderLabels({"Frame", "Worm", "Shape", "Event", "Review"});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->hide();
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_table->setMinimumHeight(65);
    m_table->setToolTip("Click a row to select its worm and seek to its frame.");
    layout->addWidget(m_table, 1);
    connect(m_table, &QTableWidget::cellClicked, this, [this](int row, int) {
        finishDrag();
        const int frame = m_table->item(row, 0)->data(Qt::UserRole).toInt();
        m_worm->setValue(m_table->item(row, 1)->data(Qt::UserRole).toInt());
        m_video->pause();
        m_video->seekToFrame(frame);
    });

    auto* files = new QHBoxLayout;
    auto* mapping = button(files, "Mapping…", "referenceSourceMapping", [this] { editSourceMapping(); });
    mapping->setToolTip("Record the original video, crop offset, and first original frame.");
    button(files, "Load…", "referenceLoad", [this] { loadFile(); });
    button(files, "Save as…", "referenceSaveAs", [this] { saveAs(); });
    outer->addLayout(files);
    m_status = new QLabel(this);
    m_status->setObjectName("referenceSaveStatus");
    m_status->setWordWrap(true);
    m_status->setTextInteractionFlags(Qt::TextSelectableByMouse);
    outer->addWidget(m_status);
    setEnabled(false);

    connect(m_worm, &QSpinBox::valueChanged, this, [this] { finishDrag(); refresh(); });
    connect(m_tool, &QComboBox::currentIndexChanged, this, [this] { finishDrag(); selectTool(); });
    connect(video, &VideoLoader::videoLoadStarted, this, [this] { setEnabled(false); });
    connect(video, &VideoLoader::videoLoaded, this, &ReferenceAnnotationPanel::openVideo);
    connect(video, &VideoLoader::framePresented, this, [this] { finishDrag(); if (m_active) refresh(); });
    connect(video, &VideoLoader::interactionModeChanged, this, [this](VideoLoader::InteractionMode mode) {
        if (m_active && mode == VideoLoader::InteractionMode::PanZoom) m_tool->setCurrentIndex(Pan);
    });
    connect(video, &VideoLoader::referencePointerPressed, this, &ReferenceAnnotationPanel::pointerPressed);
    connect(video, &VideoLoader::referencePointerMoved, this, &ReferenceAnnotationPanel::pointerMoved);
    connect(video, &VideoLoader::referencePointerReleased, this, &ReferenceAnnotationPanel::finishDrag);
    auto shortcut = [this](const QKeySequence& key, std::function<void()> action) {
        auto* s = new QShortcut(key, this);
        s->setContext(Qt::WindowShortcut);
        s->setEnabled(false);
        connect(s, &QShortcut::activated, this, std::move(action));
        m_shortcuts.append(s);
    };
    shortcut(QKeySequence::Undo, [this] { finishDrag(); m_undo.undo(); });
    shortcut(QKeySequence::Redo, [this] { finishDrag(); m_undo.redo(); });
    shortcut(QKeySequence(Qt::Key_Comma), [this] { navigate(-1, false); });
    shortcut(QKeySequence(Qt::Key_Period), [this] { navigate(1, false); });
    shortcut(QKeySequence(Qt::SHIFT | Qt::Key_Comma), [this] { navigate(-1, true); });
    shortcut(QKeySequence(Qt::SHIFT | Qt::Key_Period), [this] { navigate(1, true); });
    shortcut(QKeySequence(Qt::Key_P), [this] { m_tool->setCurrentIndex(Position); });
    shortcut(QKeySequence(Qt::Key_A), [this] { m_tool->setCurrentIndex(EndA); });
    shortcut(QKeySequence(Qt::Key_B), [this] { m_tool->setCurrentIndex(EndB); });
    shortcut(QKeySequence(Qt::Key_C), [this] { m_tool->setCurrentIndex(Centerline); });
    shortcut(QKeySequence(Qt::Key_Escape), [this] { m_tool->setCurrentIndex(Pan); });
    connect(qApp, &QApplication::focusChanged, this, [this] { updateShortcuts(); });
}

int ReferenceAnnotationPanel::frameNumber() const { return m_video->getCurrentFrameNumber(); }

void ReferenceAnnotationPanel::setActive(bool active) {
    if (active == m_active) { updateShortcuts(); return; }
    finishDrag();
    if (active && !m_active) m_savedMode = static_cast<int>(m_video->getCurrentInteractionMode());
    m_active = active;
    m_video->setReferenceViewActive(active);
    if (active) {
        m_video->pause();
        refresh();
        selectTool();
    } else if (m_video->getCurrentInteractionMode() == VideoLoader::InteractionMode::ReferenceAnnotation ||
               m_video->getCurrentInteractionMode() == VideoLoader::InteractionMode::PanZoom) {
        m_video->setInteractionMode(static_cast<VideoLoader::InteractionMode>(m_savedMode));
    }
    updateShortcuts();
}

void ReferenceAnnotationPanel::updateShortcuts() {
    auto* focus = QApplication::focusWidget();
    const bool editingText = QApplication::activePopupWidget() || qobject_cast<QLineEdit*>(focus) ||
        qobject_cast<QAbstractSpinBox*>(focus) || qobject_cast<QComboBox*>(focus);
    for (auto* s : m_shortcuts) s->setEnabled(m_active && isVisible() && isEnabled() && !editingText);
}

void ReferenceAnnotationPanel::selectTool() {
    if (!m_active) return;
    m_video->setScaleMeasureMode(false);
    m_video->pause();
    m_video->setInteractionMode(m_tool->currentIndex() == Pan ? VideoLoader::InteractionMode::PanZoom : VideoLoader::InteractionMode::ReferenceAnnotation);
}

void ReferenceAnnotationPanel::openVideo(const QString& path, int frames, double fps, QSize size) {
    m_undo.clear();
    m_document = Document{};
    m_document.video = {QFileInfo(path).absoluteFilePath(), size, frames, fps, QFileInfo(path).size()};
    m_document.source.path = m_document.video.path;
    m_path = defaultPath(m_document.video.path);
    m_dirty = false;
    m_error.clear();
    if (QFileInfo::exists(m_path)) {
        Document loaded;
        if (ReferenceAnnotations::load(m_path, m_document.video, &loaded, &m_error)) m_document = std::move(loaded);
        else {
            m_error = "Existing reference file could not be loaded: " + m_error + " Use Load or Save as to recover; the existing file will be preserved.";
            m_path.clear();
        }
    }
    setEnabled(true);
    refresh();
    selectTool();
    updateShortcuts();
}

void ReferenceAnnotationPanel::refreshOverlay() {
    m_video->setReferenceOverlay(m_document.frames.value(frameNumber()), m_worm->value());
}

void ReferenceAnnotationPanel::refresh() {
    m_refreshing = true;
    const auto a = m_document.frames.value(frameNumber()).value(m_worm->value());
    for (const auto& pair : {qMakePair(m_orientation, a.orientation), qMakePair(m_visibility, a.visibility),
                            qMakePair(m_event, a.event), qMakePair(m_review, a.review)})
        pair.first->setCurrentIndex(pair.first->findData(pair.second));
    if (m_notes->text() != a.notes) m_notes->setText(a.notes);
    m_frameLabel->setText(QString("Frame %1").arg(frameNumber()));
    m_frameLabel->setToolTip(QString("%1 marked frames").arg(m_document.frames.size()));
    m_table->setRowCount(0);
    for (auto f = m_document.frames.cbegin(); f != m_document.frames.cend(); ++f) {
        for (auto w = f->cbegin(); w != f->cend(); ++w) {
            const int row = m_table->rowCount();
            m_table->insertRow(row);
            QStringList geometry;
            if (w->position) geometry << "P";
            if (w->endA) geometry << "A";
            if (w->endB) geometry << "B";
            if (!w->centerline.isEmpty()) geometry << QString("Line (%1)").arg(w->centerline.size());
            const QString event = m_event->itemText(m_event->findData(w->event));
            const QString review = m_review->itemText(m_review->findData(w->review));
            const QStringList cells{QString::number(f.key()), QString::number(w.key()), geometry.join(", "), event, review};
            for (int col = 0; col < cells.size(); ++col) {
                auto* item = new QTableWidgetItem(cells[col]);
                item->setToolTip(cells[col]);
                if (col < 2) item->setData(Qt::UserRole, col == 0 ? f.key() : w.key());
                m_table->setItem(row, col, item);
            }
            if (f.key() == frameNumber() && w.key() == m_worm->value()) m_table->selectRow(row);
        }
    }
    m_status->setText(m_error.isEmpty() ? (m_path.isEmpty() ? "Choose Save as to enable autosave." :
        QString("%1: %2").arg(m_dirty ? "Unsaved" : QFileInfo::exists(m_path) ? "Autosaved" : "Autosave on first edit", QFileInfo(m_path).fileName())) : m_error);
    m_status->setToolTip(m_path);
    m_status->setStyleSheet(m_error.isEmpty() ? "" : "color: #c43e3e;");
    refreshOverlay();
    m_refreshing = false;
}

void ReferenceAnnotationPanel::apply(const Document& document) {
    m_document = document;
    m_dirty = true;
    save();
    refresh();
}

void ReferenceAnnotationPanel::commit(const Document& before, const QString& label) {
    if (before == m_document) { refresh(); return; }
    m_undo.push(new Edit(before, m_document, [this](const Document& d) { apply(d); }, label));
}

bool ReferenceAnnotationPanel::save() {
    if (!m_dirty) return true;
    if (m_path.isEmpty()) { m_error = "Annotations are unsaved. Choose Save as to save them without replacing the unreadable reference file."; return false; }
    if (!ReferenceAnnotations::save(m_path, m_document, &m_error)) {
        m_error = "Autosave failed: " + m_error + " Choose Save as to save elsewhere.";
        return false;
    }
    m_dirty = false;
    m_error.clear();
    return true;
}

void ReferenceAnnotationPanel::saveAs() {
    finishDrag();
    const QString path = QFileDialog::getSaveFileName(this, "Save reference annotations", m_path.isEmpty() ? defaultPath(m_document.video.path) : m_path, "JSON files (*.json)");
    if (path.isEmpty()) return;
    QString error;
    if (!ReferenceAnnotations::save(path, m_document, &error)) {
        QMessageBox::warning(this, "Save reference annotations", error);
        return;
    }
    m_path = path;
    m_dirty = false;
    m_error.clear();
    refresh();
}

bool ReferenceAnnotationPanel::flushBeforeLeaving() {
    finishDrag();
    if (save()) return true;
    const auto answer = QMessageBox::warning(this, "Unsaved reference annotations", m_error,
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (answer == QMessageBox::Save) { saveAs(); return !m_dirty; }
    if (answer == QMessageBox::Discard) { m_dirty = false; return true; }
    return false;
}

void ReferenceAnnotationPanel::loadFile() {
    if (!flushBeforeLeaving()) return;
    const QString path = QFileDialog::getOpenFileName(this, "Load reference annotations", QFileInfo(m_document.video.path).absolutePath(), "JSON files (*.json)");
    if (path.isEmpty()) return;
    Document loaded;
    QString error;
    if (!ReferenceAnnotations::load(path, m_document.video, &loaded, &error)) {
        QMessageBox::warning(this, "Load reference annotations", error);
        return;
    }
    m_undo.clear();
    m_document = std::move(loaded);
    m_path = path;
    m_dirty = false;
    m_error.clear();
    refresh();
}

void ReferenceAnnotationPanel::changeLabel() {
    if (m_refreshing || !m_active || !isEnabled() || frameNumber() < 0) return;
    finishDrag();
    const auto before = m_document;
    const auto old = m_document.frames.value(frameNumber()).value(m_worm->value());
    Frame a = old;
    a.orientation = m_orientation->currentData().toString();
    a.visibility = m_visibility->currentData().toString();
    a.event = m_event->currentData().toString();
    a.notes = m_notes->text();
    if (!(a == old)) a.review = "unreviewed";
    else a.review = m_review->currentData().toString();
    if (a == old) return;
    m_document.frames[frameNumber()][m_worm->value()] = a;
    commit(before, "Edit reference labels");
}

void ReferenceAnnotationPanel::pointerPressed(QPointF p, bool remove) {
    if (!m_active || !isEnabled() || frameNumber() < 0) return;
    finishDrag();
    const int tool = m_tool->currentIndex();
    if (tool == Pan) return;
    m_dragBefore = m_document;
    m_dragFrame = frameNumber();
    m_dragWorm = m_worm->value();
    m_dragVertex = -1;
    if (tool == Centerline) {
        const auto line = m_document.frames.value(m_dragFrame).value(m_dragWorm).centerline;
        double distance = 8.0 * m_video->videoPixelsPerWidgetPixel();
        for (int i = 0; i < line.size(); ++i) {
            const double d = std::hypot(line[i].x() - p.x(), line[i].y() - p.y());
            if (d <= distance) { distance = d; m_dragVertex = i; }
        }
        if (remove) {
            if (m_dragVertex >= 0) {
                auto& a = m_document.frames[m_dragFrame][m_dragWorm];
                a.centerline.removeAt(m_dragVertex);
                a.review = "unreviewed";
                commit(m_dragBefore, "Remove centerline vertex");
            }
            return;
        }
        if (m_dragVertex < 0) {
            auto& lineRef = m_document.frames[m_dragFrame][m_dragWorm].centerline;
            m_dragVertex = lineRef.size();
            lineRef.append(p);
        }
    } else if (remove) return;
    m_dragging = true;
    pointerMoved(p);
}

void ReferenceAnnotationPanel::pointerMoved(QPointF p) {
    if (!m_dragging) return;
    auto& a = m_document.frames[m_dragFrame][m_dragWorm];
    switch (m_tool->currentIndex()) {
    case Position: a.position = p; break;
    case EndA: a.endA = p; break;
    case EndB: a.endB = p; break;
    case Centerline: a.centerline[m_dragVertex] = p; break;
    default: return;
    }
    a.review = "unreviewed";
    refreshOverlay();
}

void ReferenceAnnotationPanel::finishDrag() {
    if (!m_dragging) return;
    m_dragging = false;
    commit(m_dragBefore, "Edit reference geometry");
}

void ReferenceAnnotationPanel::clearTool() {
    finishDrag();
    const auto before = m_document;
    if (!m_document.frames.value(frameNumber()).contains(m_worm->value())) return;
    auto& a = m_document.frames[frameNumber()][m_worm->value()];
    switch (m_tool->currentIndex()) {
    case Position: a.position.reset(); break;
    case EndA: a.endA.reset(); break;
    case EndB: a.endB.reset(); break;
    case Centerline: a.centerline.clear(); break;
    default: return;
    }
    a.review = "unreviewed";
    commit(before, "Clear reference geometry");
}

void ReferenceAnnotationPanel::deleteFrame() {
    finishDrag();
    const auto before = m_document;
    const int f = frameNumber();
    if (!m_document.frames.value(f).contains(m_worm->value())) return;
    m_document.frames[f].remove(m_worm->value());
    if (m_document.frames[f].isEmpty()) m_document.frames.remove(f);
    commit(before, "Delete worm/frame reference");
}

void ReferenceAnnotationPanel::swapEnds() {
    finishDrag();
    const auto before = m_document;
    if (!m_document.frames.value(frameNumber()).contains(m_worm->value())) return;
    auto& a = m_document.frames[frameNumber()][m_worm->value()];
    std::swap(a.endA, a.endB);
    std::reverse(a.centerline.begin(), a.centerline.end());
    if (a.orientation == "a_is_head") a.orientation = "b_is_head";
    else if (a.orientation == "b_is_head") a.orientation = "a_is_head";
    a.review = "unreviewed";
    commit(before, "Swap reference ends");
}

void ReferenceAnnotationPanel::navigate(int direction, bool annotated) {
    finishDrag();
    m_video->pause();
    if (!annotated) {
        const qint64 target = static_cast<qint64>(frameNumber()) + direction * m_step->value();
        m_video->seekToFrame(static_cast<int>(qBound(qint64(0), target, qint64(m_document.video.frameCount - 1))));
        return;
    }
    // Navigate marked frames for the currently selected reference worm only.
    const auto keys = m_document.frames.keys();
    const auto bounds = m_video->getFrameCrop();
    int target = -1;
    for (int f : keys) {
        if (f < bounds.first || f > bounds.second) continue;
        if (!m_document.frames[f].contains(m_worm->value())) continue;
        if (direction > 0 && f > frameNumber()) { target = f; break; }
        if (direction < 0 && f < frameNumber()) target = f;
    }
    if (target >= 0) m_video->seekToFrame(target);
}

void ReferenceAnnotationPanel::editSourceMapping() {
    finishDrag();
    QDialog dialog(this);
    dialog.setWindowTitle("Original video and crop mapping");
    QFormLayout form(&dialog);
    auto* path = new QLineEdit(m_document.source.path, &dialog);
    auto* x = new QSpinBox(&dialog);
    auto* y = new QSpinBox(&dialog);
    auto* frame = new QSpinBox(&dialog);
    x->setRange(0, std::numeric_limits<int>::max());
    y->setRange(0, std::numeric_limits<int>::max());
    frame->setRange(0, std::numeric_limits<int>::max() - m_document.video.frameCount);
    x->setValue(m_document.source.cropOrigin.x());
    y->setValue(m_document.source.cropOrigin.y());
    frame->setValue(m_document.source.firstFrame);
    auto* explanation = new QLabel("References use loaded-video pixels and zero-based frames. Original X/Y = reference X/Y + crop offset; original frame = reference frame + first frame. This assumes the original frame rate was preserved.", &dialog);
    explanation->setWordWrap(true);
    form.addRow(explanation);
    form.addRow("Original video", path);
    form.addRow("Crop X", x);
    form.addRow("Crop Y", y);
    form.addRow("First original frame", frame);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    form.addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    if (dialog.exec() != QDialog::Accepted) return;
    const auto before = m_document;
    m_document.source = {path->text(), QPoint(x->value(), y->value()), frame->value()};
    commit(before, "Edit source-video mapping");
}

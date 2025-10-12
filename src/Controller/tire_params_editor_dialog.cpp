#include "src/Controller/tire_params_editor_dialog.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QMessageBox>
#include <QLineEdit>
#include <QDoubleValidator> 
#include <QLocale>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QLabel>
#include <QString>

// Helper alias for readability
using MemberPtr = double PacejkaParams::*;

TireParamsEditorDialog::TireParamsEditorDialog(QWidget* parent)
    : QDialog(parent), m_params{}, m_isModified{false}
{
    // initialize defaults
    buildUi();
    setParams(m_params);
    m_isModified = false;
}

TireParamsEditorDialog::TireParamsEditorDialog(const PacejkaParams& p, QWidget* parent)
    : QDialog(parent), m_params{}, m_isModified{false}
{
    // Initialization of the new window
    buildUi();
    setParams(m_params);
    m_isModified = false;
}

void TireParamsEditorDialog::buildUi()
{
    // Window build
    // top-level layout
    QVBoxLayout* mainLay = new QVBoxLayout(this);

    // scroll area for many fields
    QScrollArea* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    QWidget* container = new QWidget(scroll);
    QVBoxLayout* containerLay = new QVBoxLayout(container);

    // helper lambda to create a group and return its form layout
    auto makeGroup = [&](const QString& title)->QFormLayout* {
        QGroupBox* g = new QGroupBox(title, container);
        QFormLayout* form = new QFormLayout(g);
        g->setLayout(form);
        containerLay->addWidget(g);
        return form;
    };

    // Create groups
    QFormLayout* longLayout = makeGroup("Longitudinal parameters (x)");
    QFormLayout* latLayout  = makeGroup("Lateral parameters (y)");
    QFormLayout* alignLayout = makeGroup("Aligning moment (z)");
    QFormLayout* scaleLayout = makeGroup("Scaling factors");

    // internal helper to add a field to a specific form
    auto addField = [&](QFormLayout* form, const QString& key, const QString& label, MemberPtr ptr,
                        double min = -1e9, double max = 1e9, double step = 0.01, int decimals = 6,
                        const QString& tooltip = QString()) {
        QLineEdit* edit = new QLineEdit();
        
        // Add a validator to ensure only numbers can be entered
        QRegularExpression rx("^-?[0-9]*\\.?[0-9]*$");
        QValidator* validator = new QRegularExpressionValidator(rx, edit);
        edit->setValidator(validator);

        edit->setToolTip(tooltip);
        
        // initial value (from m_params if possible), converting double to QString
        edit->setText(QString::number(m_params.*ptr));
        
        form->addRow(label, edit);
        m_fields.insert(key, edit);
        m_memberMap.insert(key, ptr);
        connect(edit, &QLineEdit::textChanged, this, &TireParamsEditorDialog::onModification);
    };

    // -------------------------
    // NAME: add field list
    // -------------------------
    QHBoxLayout* nameLay = new QHBoxLayout();
    QLabel* nameLabel = new QLabel("Tire Name:");
    m_nameEdit = new QLineEdit();
    connect(m_nameEdit, &QLineEdit::textChanged, this, &TireParamsEditorDialog::onModification); 
    nameLay->addWidget(nameLabel);
    nameLay->addWidget(m_nameEdit);
    mainLay->addLayout(nameLay);

    // -------------------------
    // LONGITUDINAL: add field list
    // -------------------------
    addField(longLayout, "p_Cx1", "p_Cx1", &PacejkaParams::p_Cx1, -1e6, 1e6, 0.01, 6);
    addField(longLayout, "p_Dx1", "p_Dx1", &PacejkaParams::p_Dx1);
    addField(longLayout, "p_Dx2", "p_Dx2", &PacejkaParams::p_Dx2);
    addField(longLayout, "p_Dx3", "p_Dx3", &PacejkaParams::p_Dx3);
    addField(longLayout, "p_Ex1", "p_Ex1", &PacejkaParams::p_Ex1);
    addField(longLayout, "p_Ex2", "p_Ex2", &PacejkaParams::p_Ex2);
    addField(longLayout, "p_Ex3", "p_Ex3", &PacejkaParams::p_Ex3);
    addField(longLayout, "p_Ex4", "p_Ex4", &PacejkaParams::p_Ex4);
    addField(longLayout, "p_Kx1", "p_Kx1", &PacejkaParams::p_Kx1);
    addField(longLayout, "p_Kx2", "p_Kx2", &PacejkaParams::p_Kx2);
    addField(longLayout, "p_Kx3", "p_Kx3", &PacejkaParams::p_Kx3);
    addField(longLayout, "p_Hx1", "p_Hx1", &PacejkaParams::p_Hx1);
    addField(longLayout, "p_Hx2", "p_Hx2", &PacejkaParams::p_Hx2);
    addField(longLayout, "p_Vx1", "p_Vx1", &PacejkaParams::p_Vx1);
    addField(longLayout, "p_Vx2", "p_Vx2", &PacejkaParams::p_Vx2);
    addField(longLayout, "r_Bx1", "r_Bx1", &PacejkaParams::r_Bx1);
    addField(longLayout, "r_Bx2", "r_Bx2", &PacejkaParams::r_Bx2);
    addField(longLayout, "r_Cx1", "r_Cx1", &PacejkaParams::r_Cx1);
    addField(longLayout, "r_Ex1", "r_Ex1", &PacejkaParams::r_Ex1);
    addField(longLayout, "r_Ex2", "r_Ex2", &PacejkaParams::r_Ex2);
    addField(longLayout, "r_Hx1", "r_Hx1", &PacejkaParams::r_Hx1);

    // -------------------------
    // LATERAL: add many fields
    // -------------------------
    addField(latLayout, "p_Cy1", "p_Cy1", &PacejkaParams::p_Cy1);
    addField(latLayout, "p_Dy1", "p_Dy1", &PacejkaParams::p_Dy1);
    addField(latLayout, "p_Dy2", "p_Dy2", &PacejkaParams::p_Dy2);
    addField(latLayout, "p_Dy3", "p_Dy3", &PacejkaParams::p_Dy3);
    addField(latLayout, "p_Ey1", "p_Ey1", &PacejkaParams::p_Ey1);
    addField(latLayout, "p_Ey2", "p_Ey2", &PacejkaParams::p_Ey2);
    addField(latLayout, "p_Ey3", "p_Ey3", &PacejkaParams::p_Ey3);
    addField(latLayout, "p_Ey4", "p_Ey4", &PacejkaParams::p_Ey4);
    addField(latLayout, "p_Ky1", "p_Ky1", &PacejkaParams::p_Ky1);
    addField(latLayout, "p_Ky2", "p_Ky2", &PacejkaParams::p_Ky2);
    addField(latLayout, "p_Ky3", "p_Ky3", &PacejkaParams::p_Ky3);
    addField(latLayout, "p_Hy1", "p_Hy1", &PacejkaParams::p_Hy1);
    addField(latLayout, "p_Hy2", "p_Hy2", &PacejkaParams::p_Hy2);
    addField(latLayout, "p_Hy3", "p_Hy3", &PacejkaParams::p_Hy3);
    addField(latLayout, "p_Vy1", "p_Vy1", &PacejkaParams::p_Vy1);
    addField(latLayout, "p_Vy2", "p_Vy2", &PacejkaParams::p_Vy2);
    addField(latLayout, "p_Vy3", "p_Vy3", &PacejkaParams::p_Vy3);
    addField(latLayout, "p_Vy4", "p_Vy4", &PacejkaParams::p_Vy4);
    addField(latLayout, "r_By1", "r_By1", &PacejkaParams::r_By1);
    addField(latLayout, "r_By2", "r_By2", &PacejkaParams::r_By2);
    addField(latLayout, "r_By3", "r_By3", &PacejkaParams::r_By3);
    addField(latLayout, "r_Cy1", "r_Cy1", &PacejkaParams::r_Cy1);
    addField(latLayout, "r_Ey1", "r_Ey1", &PacejkaParams::r_Ey1);
    addField(latLayout, "r_Ey2", "r_Ey2", &PacejkaParams::r_Ey2);
    addField(latLayout, "r_Hy1", "r_Hy1", &PacejkaParams::r_Hy1);
    addField(latLayout, "r_Hy2", "r_Hy2", &PacejkaParams::r_Hy2);
    addField(latLayout, "r_Vy1", "r_Vy1", &PacejkaParams::r_Vy1);
    addField(latLayout, "r_Vy2", "r_Vy2", &PacejkaParams::r_Vy2);
    addField(latLayout, "r_Vy3", "r_Vy3", &PacejkaParams::r_Vy3);
    addField(latLayout, "r_Vy4", "r_Vy4", &PacejkaParams::r_Vy4);
    addField(latLayout, "r_Vy5", "r_Vy5", &PacejkaParams::r_Vy5);
    addField(latLayout, "r_Vy6", "r_Vy6", &PacejkaParams::r_Vy6);

    // -------------------------
    // ALIGNING MOMENT: fields
    // -------------------------
    addField(alignLayout, "q_Bz1", "q_Bz1", &PacejkaParams::q_Bz1);
    addField(alignLayout, "q_Bz2", "q_Bz2", &PacejkaParams::q_Bz2);
    addField(alignLayout, "q_Bz3", "q_Bz3", &PacejkaParams::q_Bz3);
    addField(alignLayout, "q_Bz4", "q_Bz4", &PacejkaParams::q_Bz4);
    addField(alignLayout, "q_Bz5", "q_Bz5", &PacejkaParams::q_Bz5);
    addField(alignLayout, "q_Bz9", "q_Bz9", &PacejkaParams::q_Bz9);
    addField(alignLayout, "q_Bz10", "q_Bz10", &PacejkaParams::q_Bz10);
    addField(alignLayout, "q_Cz1", "q_Cz1", &PacejkaParams::q_Cz1);
    addField(alignLayout, "q_Dz1", "q_Dz1", &PacejkaParams::q_Dz1);
    addField(alignLayout, "q_Dz2", "q_Dz2", &PacejkaParams::q_Dz2);
    addField(alignLayout, "q_Dz3", "q_Dz3", &PacejkaParams::q_Dz3);
    addField(alignLayout, "q_Dz4", "q_Dz4", &PacejkaParams::q_Dz4);
    addField(alignLayout, "q_Dz6", "q_Dz6", &PacejkaParams::q_Dz6);
    addField(alignLayout, "q_Dz7", "q_Dz7", &PacejkaParams::q_Dz7);
    addField(alignLayout, "q_Dz8", "q_Dz8", &PacejkaParams::q_Dz8);
    addField(alignLayout, "q_Dz9", "q_Dz9", &PacejkaParams::q_Dz9);
    addField(alignLayout, "q_Ez1", "q_Ez1", &PacejkaParams::q_Ez1);
    addField(alignLayout, "q_Ez2", "q_Ez2", &PacejkaParams::q_Ez2);
    addField(alignLayout, "q_Ez3", "q_Ez3", &PacejkaParams::q_Ez3);
    addField(alignLayout, "q_Ez4", "q_Ez4", &PacejkaParams::q_Ez4);
    addField(alignLayout, "q_Ez5", "q_Ez5", &PacejkaParams::q_Ez5);
    addField(alignLayout, "q_Hz1", "q_Hz1", &PacejkaParams::q_Hz1);
    addField(alignLayout, "q_Hz2", "q_Hz2", &PacejkaParams::q_Hz2);
    addField(alignLayout, "q_Hz3", "q_Hz3", &PacejkaParams::q_Hz3);
    addField(alignLayout, "q_Hz4", "q_Hz4", &PacejkaParams::q_Hz4);
    addField(alignLayout, "S_Sz1", "S_Sz1", &PacejkaParams::S_Sz1);
    addField(alignLayout, "S_Sz2", "S_Sz2", &PacejkaParams::S_Sz2);
    addField(alignLayout, "S_Sz3", "S_Sz3", &PacejkaParams::S_Sz3);
    addField(alignLayout, "S_Sz4", "S_Sz4", &PacejkaParams::S_Sz4);

    // -------------------------
    // SCALING FACTORS
    // -------------------------
    addField(scaleLayout, "lambda_gammax", "lambda_gammax", &PacejkaParams::lambda_gammax);
    addField(scaleLayout, "lambda_Cx", "lambda_Cx", &PacejkaParams::lambda_Cx);
    addField(scaleLayout, "lambda_mux", "lambda_mux", &PacejkaParams::lambda_mux);
    addField(scaleLayout, "lambda_Ex", "lambda_Ex", &PacejkaParams::lambda_Ex);
    addField(scaleLayout, "lambda_Kx", "lambda_Kx", &PacejkaParams::lambda_Kx);
    addField(scaleLayout, "lambda_Hx", "lambda_Hx", &PacejkaParams::lambda_Hx);
    addField(scaleLayout, "lambda_Vx", "lambda_Vx", &PacejkaParams::lambda_Vx);
    addField(scaleLayout, "lambda_xalpha", "lambda_xalpha", &PacejkaParams::lambda_xalpha);

    addField(scaleLayout, "lambda_muy", "lambda_muy", &PacejkaParams::lambda_muy);
    addField(scaleLayout, "lambda_Ky", "lambda_Ky", &PacejkaParams::lambda_Ky);
    addField(scaleLayout, "lambda_gammay", "lambda_gammay", &PacejkaParams::lambda_gammay);
    addField(scaleLayout, "lambda_Cy", "lambda_Cy", &PacejkaParams::lambda_Cy);
    addField(scaleLayout, "lambda_Ey", "lambda_Ey", &PacejkaParams::lambda_Ey);
    addField(scaleLayout, "lambda_Hy", "lambda_Hy", &PacejkaParams::lambda_Hy);
    addField(scaleLayout, "lambda_Vy", "lambda_Vy", &PacejkaParams::lambda_Vy);
    addField(scaleLayout, "lambda_Vykappa", "lambda_Vykappa", &PacejkaParams::lambda_Vykappa);
    addField(scaleLayout, "lambda_ykappa", "lambda_ykappa", &PacejkaParams::lambda_ykappa);

    addField(scaleLayout, "lambda_Fz0", "lambda_Fz0", &PacejkaParams::lambda_Fz0);
    addField(scaleLayout, "F_z0", "F_z0", &PacejkaParams::F_z0);
    addField(scaleLayout, "lambda_S", "lambda_S", &PacejkaParams::lambda_S);

    // finish container
    container->setLayout(containerLay);
    scroll->setWidget(container);
    mainLay->addWidget(scroll);

    // bottom buttons
    QHBoxLayout* btnLay = new QHBoxLayout();
    QPushButton* loadBtn = new QPushButton("Load JSON");
    QPushButton* saveBtn = new QPushButton("Save JSON");
    QPushButton* resetBtn = new QPushButton("Reset defaults");
    QPushButton* okBtn = new QPushButton("OK");
    QPushButton* cancelBtn = new QPushButton("Cancel");

    btnLay->addWidget(loadBtn);
    btnLay->addWidget(saveBtn);
    btnLay->addWidget(resetBtn);
    btnLay->addStretch();
    btnLay->addWidget(okBtn);
    btnLay->addWidget(cancelBtn);
    mainLay->addLayout(btnLay);

    connect(loadBtn, &QPushButton::clicked, this, &TireParamsEditorDialog::onLoadClicked);
    connect(saveBtn, &QPushButton::clicked, this, &TireParamsEditorDialog::onSaveClicked);
    connect(resetBtn, &QPushButton::clicked, this, &TireParamsEditorDialog::onResetDefaultsClicked);
    connect(okBtn, &QPushButton::clicked, this, [this]() {
        if (!validateInputs()) {
            return; // Stop if validation fails
        }
        m_params = getParams();
        accept();
    });
    connect(cancelBtn, &QPushButton::clicked, this, &TireParamsEditorDialog::reject);
}

void TireParamsEditorDialog::onLoadClicked()
{
    QString fn = QFileDialog::getOpenFileName(this, "Open tire JSON", QString(), "JSON files (*.json);;All files (*)");
    if (fn.isEmpty()) return;
    if (!loadFromJsonFile(fn)) {
        QMessageBox::warning(this, "Load error", "Could not load JSON file.");
    }else {
        m_isModified = false; 
    }
}

void TireParamsEditorDialog::onSaveClicked()
{
    if (!m_isModified) {
        return;
    }
    if (!validateInputs()) {
        return;
    }
    QString fn = QFileDialog::getSaveFileName(this, "Save tire JSON", QString(), "JSON files (*.json);;All files (*)");
    if (fn.isEmpty()) return;
    if (!saveToJsonFile(fn)) {
        QMessageBox::warning(this, "Save error", "Could not save JSON file.");
    } else {
        m_isModified = false; // Successfully saved, data is now clean
    }
}

void TireParamsEditorDialog::onResetDefaultsClicked()
{
    // optionally define some defaults; currently zeroed
    m_params = PacejkaParams();
    setParams(m_params);
    m_isModified = false;
}

PacejkaParams TireParamsEditorDialog::getParams() const
{
    PacejkaParams p = m_params; 
    QLocale cLocale(QLocale::C);
    
    for (auto it = m_memberMap.begin(); it != m_memberMap.end(); ++it) {
        const QString& key = it.key();
        MemberPtr ptr = it.value();
        if (m_fields.contains(key)) {
            QLineEdit* field = m_fields.value(key);
            p.*ptr = cLocale.toDouble(field->text());
        }
    }
    p.name = m_nameEdit->text();

    return p;
}

void TireParamsEditorDialog::setParams(const PacejkaParams& p)
{
    m_params = p;
    // populate widgets
    for (auto it = m_memberMap.begin(); it != m_memberMap.end(); ++it) {
        const QString& key = it.key();
        MemberPtr ptr = it.value();
        if (m_fields.contains(key)) {
            QLineEdit* field = m_fields[key];
            QSignalBlocker blocker(field); 
            field->setText(QString::number(m_params.*ptr));
        }
    }
    {
        QSignalBlocker blocker(m_nameEdit);
        m_nameEdit->setText(m_params.name);
    }
     m_isModified = false; 
}

bool TireParamsEditorDialog::validateInputs()
{
    for (auto it = m_fields.constBegin(); it != m_fields.constEnd(); ++it) {
        QLineEdit* field = it.value();
        if (!field->hasAcceptableInput()) {
            QMessageBox::warning(this, "Invalid Input",
                QString("The value for parameter '%1' is not a valid number.\n"
                        "Please correct it before proceeding.").arg(it.key()));
            
            field->setFocus();
            field->selectAll();
            
            return false; 
        }
    }
    
    return true; 
}


QJsonObject TireParamsEditorDialog::toJson() const
{
    QJsonObject obj;
    obj["name"] = m_nameEdit->text(); 
    QLocale cLocale(QLocale::C); // ADDED: Use the C locale for conversion

    for (auto it = m_memberMap.begin(); it != m_memberMap.end(); ++it) {
        const QString& key = it.key();
        MemberPtr ptr = it.value();
        
        double value = m_params.*ptr; // Default value
        if (m_fields.contains(key)) {
             QLineEdit* field = m_fields.value(key);
             // MODIFIED: Use the locale to convert the string to a double
             value = cLocale.toDouble(field->text());
        }
        obj.insert(key, QJsonValue(value));
    }
    return obj;
}

bool TireParamsEditorDialog::fromJson(const QJsonObject& obj)
{
    if (obj.contains("name") && obj.value("name").isString()) {
    QSignalBlocker blocker(m_nameEdit);
    m_nameEdit->setText(obj["name"].toString());
    }
    
    bool ok = true;
    for (auto it = m_memberMap.begin(); it != m_memberMap.end(); ++it) {
        const QString& key = it.key();
        if (obj.contains(key) && obj.value(key).isDouble()) {
            double v = obj.value(key).toDouble();
            if (m_fields.contains(key)) {
                QSignalBlocker blocker(m_fields[key]);
                m_fields[key]->setText(QString::number(v));
            }
        } else {
            ok = false;
        }
    }
    // update internal param snapshot
    m_params = getParams();
    m_isModified = false;
    return ok;
}

bool TireParamsEditorDialog::loadFromJsonFile(const QString& path)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    QByteArray data = f.readAll();
    f.close();
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if (err.error != QJsonParseError::NoError) return false;
    if (!doc.isObject()) return false;
    return fromJson(doc.object());
}

bool TireParamsEditorDialog::saveToJsonFile(const QString& path) const
{
    QJsonObject obj = toJson();
    QJsonDocument doc(obj);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) return false;
    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();
    return true;
}

// static convenience
bool TireParamsEditorDialog::editParams(QWidget* parent, PacejkaParams& out)
{
    TireParamsEditorDialog dlg(out, parent);
    if (dlg.exec() == QDialog::Accepted) {
        out = dlg.getParams();
        return true;
    }
    return false;
}

void TireParamsEditorDialog::onModification()
{
    m_isModified = true;
}
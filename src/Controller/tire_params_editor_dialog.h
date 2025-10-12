#pragma once
#include <QDialog>
#include <QMap>
#include <QJsonObject>
#include <QLineEdit>
#include <QDoubleValidator> 
#include "src/Model/tire_model.h"


class QDoubleSpinBox;

/**
 * @class TireParamsEditorDialog
 * @brief A dialog window for viewing and editing all coefficients of a PacejkaParams struct.
 *
 * This class provides a user-friendly interface with dynamically generated input fields
 * for all Pacejka parameters. It supports loading from and saving to JSON files,
 * input validation, and tracking of modifications.
 */
class TireParamsEditorDialog : public QDialog {
    Q_OBJECT
public:
    //! Default constructor. Creates a dialog with a default-initialized PacejkaParams object.
    explicit TireParamsEditorDialog(QWidget* parent = nullptr);
    //! Constructor that initializes the dialog with an existing PacejkaParams object.
    explicit TireParamsEditorDialog(const PacejkaParams& p, QWidget* parent = nullptr);

    //! Retrieves the current parameters from the dialog's input fields.
    PacejkaParams getParams() const;
    //! Populates the dialog's input fields with values from a PacejkaParams object.
    void setParams(const PacejkaParams& p);

    //! Loads tire parameters from a specified JSON file and updates the UI.
    bool loadFromJsonFile(const QString& path);
    //! Saves the current tire parameters from the UI to a specified JSON file.
    bool saveToJsonFile(const QString& path) const;

    /**
     * @brief A static helper function to create, show, and get results from the dialog.
     * This provides a simple one-line method to edit a PacejkaParams object.
     * @param parent The parent widget for the dialog.
     * @param outParams A reference to the PacejkaParams object to be edited.
     * @return True if the user clicked "OK" and params were accepted, false otherwise.
     */
    static bool editParams(QWidget* parent, PacejkaParams& outParams);

private slots:
    //! Handles the "Load JSON" button click.
    void onLoadClicked();
    //! Handles the "Save JSON" button click.
    void onSaveClicked();
    //! Handles the "Reset Defaults" button click, clearing all fields.
    void onResetDefaultsClicked();
    //! A slot that is triggered whenever any input field is modified, setting the `m_isModified` flag.
    void onModification();

private:
    //! Constructs the entire user interface of the dialog.
    void buildUi();
    //! Validates all input fields to ensure they contain acceptable numeric values.
    bool validateInputs();
    //! Serializes the current parameters into a QJsonObject.
    QJsonObject toJson() const;
    //! Deserializes a QJsonObject and populates the UI fields.
    bool fromJson(const QJsonObject& obj);
    QLineEdit* m_nameEdit;
    
    // Stored widgets + mapping to struct members
    QMap<QString, QLineEdit*> m_fields;                     //!< Maps a parameter key (e.g., "p_Cx1") to its corresponding QLineEdit widget.
    QMap<QString, double PacejkaParams::*> m_memberMap;     //!< Maps the same key to a pointer-to-member of the PacejkaParams struct.
    
    PacejkaParams m_params; //!< An internal snapshot of the current parameter state.
    bool m_isModified;      //!< A "dirty" flag, true if any parameter has been changed since the last save/load.
};

/*  -------- IMPROVEMENT FOR FUTURE WORKS ------------

- Separation of concers, respecting the code structure,it is
  possible to make a partition for each part of MVC. The current
  design was inteded as a separetade window single structure.
-------- ------------------------------ ------------
*/
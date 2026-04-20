#ifndef POINTWORKS_DIALOG_REGISTRY_H
#define POINTWORKS_DIALOG_REGISTRY_H

#include <map>
#include <set>
#include <QString>

namespace ct
{
    class CustomDialog;
    class CustomDock;

    class DialogRegistry
    {
    public:
        static DialogRegistry& instance();

        // 对话框管理
        void registerDialog(const QString& name, CustomDialog* dlg);
        void unregisterDialog(const QString& name);
        CustomDialog* getDialog(const QString& name);
        bool isDialogVisible(const QString& name) const;
        bool hasOpenDialog() const;

        // 停靠栏管理
        void registerDock(const QString& name, CustomDock* dock);
        void unregisterDock(const QString& name);
        CustomDock* getDock(const QString& name);
        bool isDockVisible(const QString& name) const;

        // 标签分组
        void addLeftLabel(const QString& name);
        void addRightLabel(const QString& name);
        const std::set<QString>& leftLabels() const;
        const std::set<QString>& rightLabels() const;

    private:
        DialogRegistry() = default;
        std::map<QString, CustomDialog*> m_dialogs;
        std::map<QString, bool> m_dialog_visibility;
        std::map<QString, CustomDock*> m_docks;
        std::set<QString> m_dock_visibility;
        std::set<QString> m_left_labels;
        std::set<QString> m_right_labels;
    };
}

#endif // POINTWORKS_DIALOG_REGISTRY_H

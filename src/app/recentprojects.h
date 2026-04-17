#ifndef POINTWORKS_RECENT_PROJECTS_H
#define POINTWORKS_RECENT_PROJECTS_H

#include <QObject>
#include <QStringList>
#include <QSettings>

/// 最近项目列表管理（基于 QSettings 持久化）
class RecentProjects : public QObject
{
    Q_OBJECT
public:
    static const int MAX_RECENT = 10;

    explicit RecentProjects(QObject* parent = nullptr)
        : QObject(parent), m_settings("PointWorks", "PointWorks") { load(); }

    /// 添加项目到最近列表（去重 + LRU）
    void addProject(const QString& path)
    {
        QStringList list = m_projects;
        list.removeAll(path);
        list.prepend(path);
        while (list.size() > MAX_RECENT)
            list.removeLast();
        m_projects = list;
        save();
    }

    /// 移除指定项目
    void removeProject(const QString& path)
    {
        m_projects.removeAll(path);
        save();
    }

    /// 获取列表
    QStringList projects() const { return m_projects; }

    /// 清空
    void clear()
    {
        m_projects.clear();
        save();
    }

signals:
    void changed();

private:
    void load()
    {
        m_projects = m_settings.value("recent_projects").toStringList();
    }

    void save()
    {
        m_settings.setValue("recent_projects", m_projects);
        emit changed();
    }

    QStringList m_projects;
    QSettings m_settings;
};

#endif // POINTWORKS_RECENT_PROJECTS_H

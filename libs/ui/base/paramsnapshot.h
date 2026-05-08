#ifndef PW_PARAM_SNAPSHOT_H
#define PW_PARAM_SNAPSHOT_H

#include <QMap>
#include <QString>
#include <QVariant>
#include <QVector>

#include <Eigen/Dense>

namespace pw {

class ParamSnapshot {
public:
    ParamSnapshot() = default;

    template <typename T>
    void set(const QString& key, const T& value) {
        m_params.insert(key, QVariant::fromValue(value));
    }

    void clear() { m_params.clear(); }
    int count() const { return m_params.size(); }

    bool operator==(const ParamSnapshot& other) const {
        if (m_params.size() != other.m_params.size()) return false;
        for (auto it = m_params.constBegin(); it != m_params.constEnd(); ++it) {
            auto oit = other.m_params.constFind(it.key());
            if (oit == other.m_params.constEnd() || it.value() != oit.value())
                return false;
        }
        return true;
    }
    bool operator!=(const ParamSnapshot& other) const { return !(*this == other); }

    static QString pointsToString(const QVector<Eigen::Vector3d>& points) {
        QString s;
        s.reserve(points.size() * 48);
        for (const auto& p : points)
            s += QString("%1,%2,%3;").arg(p.x(), 0, 'f', 10)
                                    .arg(p.y(), 0, 'f', 10)
                                    .arg(p.z(), 0, 'f', 10);
        return s;
    }

private:
    QMap<QString, QVariant> m_params;
};

} // namespace pw

#endif // PW_PARAM_SNAPSHOT_H

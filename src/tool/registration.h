//
// Created by LBC on 2025/1/10.
//

#ifndef POINTWORKS_REGISTRATION_H
#define POINTWORKS_REGISTRATION_H

#include "ui/base/customdialog.h"
#include "core/common.h"
#include "algorithm/registration.h"
#include "tool/descriptor.h"

#include <QFutureWatcher>
#include <QtConcurrent>


QT_BEGIN_NAMESPACE
namespace Ui {
    class Registration;
}
QT_END_NAMESPACE

class Registration : public ct::CustomDialog {
Q_OBJECT

public:
    explicit Registration(QWidget *parent = nullptr);


     ~Registration();

    // setDescriptor函数的主要功能是允许Registration类设置并管理一个Descriptor对象的引用
    void setDescriptor(Descriptor *descriptor) {
        m_descriptor = descriptor;
        // 当descriptor对象被销毁时，lambda函数将被调用，将m_descriptor成员变量设为nullptr
        if (descriptor) connect(descriptor, &Descriptor::destroyed, [=] { m_descriptor = nullptr; });
    }

    void setTarget();

    void setSource();

    void setCorrespondenceEstimation();

    void addCorrespondenceRejector();

    void removeCorrespondenceRejector();

    void setTransformationEstimation();

    void preview();

    void apply();

    void add();

    void reset();

private:
    // Build a RegistrationContext from current UI state and member variables
    ct::RegistrationContext buildContext() const;

    // Async result handlers (called on main thread via QFutureWatcher)
    void handleCorrespondenceResult(const ct::CorrespondenceResult& result);
    void handleRejectorResult(const ct::RejectorResult& result);
    void handleTransformationResult(const ct::TransformationResult& result);
    void handleRegistrationResult(const ct::RegistrationResult& result);

    Ui::Registration *ui;
    Descriptor *m_descriptor;
    ct::Cloud::Ptr m_target_cloud;
    ct::Cloud::Ptr m_source_cloud;
    ct::CorrespondencesPtr m_corr;
    ct::CorreEst::Ptr m_ce;
    ct::TransEst::Ptr m_te;
    std::map<int, ct::CorreRej::Ptr> m_cr_map;
    std::map<std::string, ct::Cloud::Ptr> m_reg_map;

    // Future watchers for async operations
    QFutureWatcher<ct::CorrespondenceResult>* m_ce_watcher = nullptr;
    QFutureWatcher<ct::RejectorResult>* m_cr_watcher = nullptr;
    QFutureWatcher<ct::TransformationResult>* m_te_watcher = nullptr;
    QFutureWatcher<ct::RegistrationResult>* m_reg_watcher = nullptr;
};


#endif //POINTWORKS_REGISTRATION_H

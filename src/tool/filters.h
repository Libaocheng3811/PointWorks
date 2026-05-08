//
// Created by LBC on 2024/12/25.
//

#ifndef TOOL_FILTERS_H
#define TOOL_FILTERS_H

#include "ui/base/customdialog.h"
#include "algorithm/filters.h"

#include <QFutureWatcher>

QT_BEGIN_NAMESPACE
namespace Ui {
    class Filters;
}
QT_END_NAMESPACE

class Filters : public pw::CustomDialog {
Q_OBJECT

public:
    explicit Filters(QWidget *parent = nullptr);

    ~Filters() override;

    void preview();

    // 将过滤后的点云（保留下的点云）添加到视图树中
    void add();

    void apply();

    virtual void reset();

private:
    void handleFilterResult(const pw::FilterResult& result);
    void runFilter(std::function<pw::FilterResult()> filterFn, bool show_progress);
    pw::ConditionBase::Ptr getCondition();
    void getRange(int index);

private:
    Ui::Filters *ui;
    std::map<std::string, pw::Cloud::Ptr> m_filter_map;
    std::atomic<bool> m_cancel{false};
    QFutureWatcher<pw::FilterResult>* m_preview_watcher = nullptr;
};


#endif //TOOL_FILTERS_H

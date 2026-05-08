#ifndef POINTWORKS_CONSOLE_H
#define POINTWORKS_CONSOLE_H

#include "core/exports.h"

#include <QTextbrowser>

namespace pw
{
    enum log_level
    {
        LOG_INFO,
        LOG_WARNING,
        LOG_ERROR,
    };

    // QTextBrowser继承自QTextEdit,
    class PW_VIZ_EXPORT Console : public QTextBrowser
    {
        Q_OBJECT
    public:
        explicit Console(QWidget *parent = nullptr) : QTextBrowser(parent) {}

        /**
         * @brief 打印日志
         * @note 函数参数的名称在函数声明中是可选的，尤其是在类的成员函数声明中。
         * 当在类的头文件中声明成员函数时，通常只写出参数类型而不写出参数名称，这是为了减少头文件中的代码冗余，并保持接口的简洁性。参数名称在类的源文件（.cpp文件）中实现函数时给出。
         */
         void print(log_level, const QString& message);
    };
}


#endif //POINTWORKS_CONSOLE_H

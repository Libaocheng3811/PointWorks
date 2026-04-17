#include "console.h"

#include <QDateTime>

namespace ct
{
    void Console::print(log_level level, const QString &message)
    {
        QString currentTime = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString level_color;
        switch (level)
        {
            case log_level::LOG_INFO:
                level_color = "<font color='black'>" + message + "</font>";
                break;
            case log_level::LOG_WARNING:
                level_color = "<font color='#CFBF17'>" + message + "</font>";
                break;
            case log_level::LOG_ERROR:
                level_color = "<font color='red'>" + message + "</font>";
                break;
        }
        append("[" + currentTime + "]:" + level_color);
        moveCursor(QTextCursor::End);
    }
}

#include "language_manager.h"
#include <QCoreApplication>

namespace pw
{

LanguageManager& LanguageManager::instance()
{
    static LanguageManager inst;
    return inst;
}

void LanguageManager::switchLanguage(LanguageManager::Language lang)
{
    if (lang == m_current) return;

    qApp->removeTranslator(&m_translator);

    if (lang == Chinese)
    {
        if (m_translator.load(":/res/trans/zh_CN.qm"))
            qApp->installTranslator(&m_translator);
    }

    m_current = lang;
    emit languageChanged(lang);
}

LanguageManager::Language LanguageManager::currentLanguage() const
{
    return m_current;
}

} // namespace pw

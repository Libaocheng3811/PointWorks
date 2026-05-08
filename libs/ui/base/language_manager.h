#pragma once

#include <QObject>
#include <QTranslator>

namespace pw
{

class LanguageManager : public QObject
{
    Q_OBJECT
public:
    enum Language { English, Chinese };

    static LanguageManager& instance();

    void switchLanguage(Language lang);
    Language currentLanguage() const;

signals:
    void languageChanged(Language lang);

private:
    LanguageManager() = default;
    QTranslator m_translator;
    Language m_current = English;
};

} // namespace pw

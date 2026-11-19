// Хранение ключа API в защищённом хранилище системы:
//   macOS   — «Связка ключей» (Keychain), служба «SDP ShipEditor»;
//   Windows — «Диспетчер учётных данных» (Credential Manager), запись «SDP ShipEditor/anthropic-api-key»;
//   Linux   — Secret Service (libsecret), если программа собрана с ним; иначе ключ хранится только до закрытия программы.
// Открытым текстом ключ не сохраняется нигде. Ключ, сохранённый прежними версиями в настройках, переносится в хранилище и стирается из настроек.
#pragma once
#include <QString>

namespace keystore {
bool persistent();                 // есть ли постоянное защищённое хранилище
QString where();                   // где хранится ключ — для подсказки пользователю
QString load();
bool save(const QString& key);     // пустой ключ — удалить
void migrateFromSettings();        // перенос ключа из QSettings("SDP","ShipEditor")/apiKey
}

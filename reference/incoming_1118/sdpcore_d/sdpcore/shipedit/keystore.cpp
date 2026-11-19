// системные заголовки — до заголовков Qt: в glib есть поле «signals», которое Qt определяет как макрос
#if defined(__APPLE__)
#include <Security/Security.h>
#include <CoreFoundation/CoreFoundation.h>
#elif defined(_WIN32)
#include <windows.h>
#include <wincred.h>
#include <string>
#elif defined(SDP_HAVE_LIBSECRET)
#include <libsecret/secret.h>
#endif
#include "keystore.h"
#include <QSettings>

[[maybe_unused]] static const char* kService = "SDP ShipEditor";
[[maybe_unused]] static const char* kAccount = "anthropic-api-key";
[[maybe_unused]] static QString g_session;          // запасной вариант без хранилища: ключ живёт до закрытия программы

#if defined(__APPLE__)
namespace {
CFStringRef cf(const char* s) { return CFStringCreateWithCString(nullptr, s, kCFStringEncodingUTF8); }
CFMutableDictionaryRef query() {
    CFMutableDictionaryRef q = CFDictionaryCreateMutable(nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFStringRef s = cf(kService), a = cf(kAccount);
    CFDictionarySetValue(q, kSecClass, kSecClassGenericPassword);
    CFDictionarySetValue(q, kSecAttrService, s);
    CFDictionarySetValue(q, kSecAttrAccount, a);
    CFRelease(s); CFRelease(a);
    return q;
}
}
bool keystore::persistent() { return true; }
QString keystore::where() { return "в «Связке ключей» macOS (запись «SDP ShipEditor»)"; }
QString keystore::load() {
    CFMutableDictionaryRef q = query();
    CFDictionarySetValue(q, kSecReturnData, kCFBooleanTrue);
    CFDictionarySetValue(q, kSecMatchLimit, kSecMatchLimitOne);
    CFTypeRef out = nullptr;
    const OSStatus st = SecItemCopyMatching(q, &out);
    CFRelease(q);
    if (st != errSecSuccess || !out) return {};
    CFDataRef d = static_cast<CFDataRef>(out);
    const QString key = QString::fromUtf8(reinterpret_cast<const char*>(CFDataGetBytePtr(d)), static_cast<int>(CFDataGetLength(d)));
    CFRelease(out);
    return key;
}
bool keystore::save(const QString& key) {
    CFMutableDictionaryRef q = query();
    SecItemDelete(q);
    bool ok = true;
    if (!key.isEmpty()) {
        const QByteArray u = key.toUtf8();
        CFDataRef d = CFDataCreate(nullptr, reinterpret_cast<const UInt8*>(u.constData()), u.size());
        CFDictionarySetValue(q, kSecValueData, d);
        CFDictionarySetValue(q, kSecAttrAccessible, kSecAttrAccessibleWhenUnlockedThisDeviceOnly);   // не уходит в iCloud и резервные копии на другие устройства
        ok = SecItemAdd(q, nullptr) == errSecSuccess;
        CFRelease(d);
    }
    CFRelease(q);
    return ok;
}

#elif defined(_WIN32)
namespace { std::wstring target() { return QString("%1/%2").arg(kService, kAccount).toStdWString(); } }
bool keystore::persistent() { return true; }
QString keystore::where() { return "в «Диспетчере учётных данных» Windows (запись «SDP ShipEditor/anthropic-api-key»)"; }
QString keystore::load() {
    PCREDENTIALW c = nullptr;
    const std::wstring t = target();
    if (!CredReadW(t.c_str(), CRED_TYPE_GENERIC, 0, &c)) return {};
    const QString key = QString::fromUtf8(reinterpret_cast<const char*>(c->CredentialBlob), static_cast<int>(c->CredentialBlobSize));
    CredFree(c);
    return key;
}
bool keystore::save(const QString& key) {
    const std::wstring t = target();
    if (key.isEmpty()) { CredDeleteW(t.c_str(), CRED_TYPE_GENERIC, 0); return true; }
    QByteArray u = key.toUtf8();
    CREDENTIALW c{};
    c.Type = CRED_TYPE_GENERIC;
    c.TargetName = const_cast<LPWSTR>(t.c_str());
    c.CredentialBlobSize = static_cast<DWORD>(u.size());
    c.CredentialBlob = reinterpret_cast<LPBYTE>(u.data());
    c.Persist = CRED_PERSIST_LOCAL_MACHINE;
    return CredWriteW(&c, 0) != FALSE;
}

#elif defined(SDP_HAVE_LIBSECRET)
namespace {
const SecretSchema* schema() {
    static const SecretSchema s = {"ru.sdp.ShipEditor", SECRET_SCHEMA_NONE,
        {{"account", SECRET_SCHEMA_ATTRIBUTE_STRING}, {nullptr, SECRET_SCHEMA_ATTRIBUTE_STRING}}, 0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};
    return &s;
}
}
bool keystore::persistent() { return true; }
QString keystore::where() { return "в хранилище паролей системы (Secret Service)"; }
QString keystore::load() {
    GError* e = nullptr;
    gchar* p = secret_password_lookup_sync(schema(), nullptr, &e, "account", kAccount, nullptr);
    if (e) { g_error_free(e); return g_session; }
    if (!p) return g_session;
    const QString key = QString::fromUtf8(p); secret_password_free(p);
    return key;
}
bool keystore::save(const QString& key) {
    GError* e = nullptr;
    if (key.isEmpty()) { secret_password_clear_sync(schema(), nullptr, &e, "account", kAccount, nullptr); if (e) g_error_free(e); g_session.clear(); return true; }
    const bool ok = secret_password_store_sync(schema(), SECRET_COLLECTION_DEFAULT, "SDP ShipEditor: ключ Anthropic API", key.toUtf8().constData(), nullptr, &e, "account", kAccount, nullptr);
    if (e) { g_error_free(e); g_session = key; return false; }   // хранилище недоступно: держим ключ до закрытия программы
    return ok;
}

#else
bool keystore::persistent() { return false; }
QString keystore::where() { return "только до закрытия программы (защищённое хранилище в этой сборке недоступно)"; }
QString keystore::load() { return g_session; }
bool keystore::save(const QString& key) { g_session = key; return true; }
#endif

void keystore::migrateFromSettings() {
    QSettings st("SDP", "ShipEditor");
    if (!st.contains("apiKey")) return;
    const QString old = st.value("apiKey").toString();
    if (!old.isEmpty() && load().isEmpty()) save(old);
    st.remove("apiKey");                // открытым текстом больше не храним
    st.sync();
}

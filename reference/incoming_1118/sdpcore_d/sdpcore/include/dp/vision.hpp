// Распознавание чертежа или фотографии судна через Claude (Anthropic API): запрос и разбор ответа.
// Сама передача по сети — в программе (sdp_ship, через Qt Network). Здесь только содержимое: что спросить и как понять ответ.
// Claude возвращает простые фигуры в пикселях присланной картинки и размеры, если они подписаны на чертеже.
// ⚠ Это подсказка: фигуры и размеры нужно проверить и поправить.
#pragma once
#include <string>
#include <vector>

#include "dp/json.hpp"
#include "dp/shipdesign.hpp"

namespace dp {
namespace vision {

struct Figure { std::string name; std::vector<std::pair<double, double>> px; };   // точки в пикселях присланной картинки
struct Answer {
    bool ok = false; std::string error;
    std::vector<Figure> figures;
    std::vector<std::pair<std::string, double>> dims;                              // подпись книги → значение (только прочитанные с чертежа)
    std::string notes;
    bool hasWaterline = false; double waterlineY = 0;                               // ватерлиния, пиксель по вертикали (вид сбоку и спереди)
};

inline std::string prompt(int view, int imgW, int imgH) {
    const char* what = view == design::Top ? "вид сверху (план палубы или корпуса): контур корпуса по ватерлинии или по палубе"
                     : view == design::Side ? "вид сбоку (профиль): корпус, надстройки, рубка, мачты, краны и другие крупные части над водой; если видна ватерлиния — подводная часть корпуса"
                                            : "вид спереди или сзади: корпус и надстройки над водой";
    return std::string("На картинке ") + what + " судна. Размер картинки " + std::to_string(imgW) + "×" + std::to_string(imgH) + " пикселей, начало в левом верхнем углу, y вниз.\n"
        "Опиши судно простыми фигурами (многоугольники до 12 вершин; прямоугольники — 4 вершины), которые вместе покрывают силуэт. "
        "Мелкие детали (леера, антенны тоньше 1 % длины) не обводи. Фигуры могут перекрываться.\n"
        "Если на картинке подписаны размеры (длина, ширина, осадка, водоизмещение), выпиши их. Ничего не придумывай: только то, что прочитано.\n"
        "Ответь ТОЛЬКО объектом JSON без пояснений вокруг:\n"
        "{\"figures\":[{\"name\":\"корпус\",\"points\":[[x,y],...]}],"
        "\"waterline_y\":число или null,"
        "\"dimensions\":{\"length_m\":число или null,\"beam_m\":число или null,\"draft_m\":число или null,\"displacement_t\":число или null},"
        "\"notes\":\"коротко по-русски: что не удалось распознать\"}";
}

// Тело запроса к /v1/messages. mediaType — «image/jpeg» или «image/png», data — картинка в base64.
inline std::string requestBody(const std::string& model, int view, int imgW, int imgH, const std::string& mediaType, const std::string& base64) {
    return "{\"model\":\"" + design::esc(model) + "\",\"max_tokens\":4096,\"messages\":[{\"role\":\"user\",\"content\":["
           "{\"type\":\"image\",\"source\":{\"type\":\"base64\",\"media_type\":\"" + mediaType + "\",\"data\":\"" + base64 + "\"}},"
           "{\"type\":\"text\",\"text\":\"" + design::esc([&] { std::string p = prompt(view, imgW, imgH), o; for (char c : p) { if (c == '\n') o += "\\n"; else o += c; } return o; }()) + "\"}]}]}";
}

// Разбор ответа API: текст первого блока, в нём — объект JSON (возможно, в ограждении ```).
inline Answer parseResponse(const std::string& body) {
    Answer a; json::Value root; std::string err;
    if (!json::parse(body, root, err)) { a.error = "ответ сервера не разобран: " + err; return a; }
    if (const json::Value* e = root.find("error")) { const json::Value* m = e->find("message"); a.error = "сервер: " + (m && m->t == json::Value::Str ? m->s : std::string("ошибка")); return a; }
    const json::Value* content = root.find("content");
    std::string text;
    if (content && content->t == json::Value::Arr) for (const auto& c : content->a) { const json::Value* t = c.find("text"); if (t && t->t == json::Value::Str) text += t->s; }
    if (text.empty()) { a.error = "в ответе нет текста"; return a; }
    const size_t b = text.find('{'), e = text.rfind('}');
    if (b == std::string::npos || e == std::string::npos || e < b) { a.error = "в ответе нет объекта JSON"; return a; }
    json::Value v; if (!json::parse(text.substr(b, e - b + 1), v, err)) { a.error = "JSON в ответе не разобран: " + err; return a; }
    if (const json::Value* fs = v.find("figures"); fs && fs->t == json::Value::Arr)
        for (const auto& f : fs->a) {
            Figure g; if (const json::Value* n = f.find("name"); n && n->t == json::Value::Str) g.name = n->s;
            if (const json::Value* ps = f.find("points"); ps && ps->t == json::Value::Arr)
                for (const auto& p : ps->a) if (p.t == json::Value::Arr && p.a.size() == 2 && p.a[0].t == json::Value::Num && p.a[1].t == json::Value::Num) g.px.push_back({p.a[0].n, p.a[1].n});
            if (g.px.size() >= 3) a.figures.push_back(g);
        }
    if (const json::Value* w = v.find("waterline_y"); w && w->t == json::Value::Num) { a.hasWaterline = true; a.waterlineY = w->n; }
    if (const json::Value* d = v.find("dimensions"); d && d->t == json::Value::Obj) {
        const std::pair<const char*, const char*> keys[] = {{"length_m", "Длина по ватерлинии"}, {"beam_m", "Ширина"}, {"draft_m", "Осадка"}, {"displacement_t", "Водоизмещение"}};
        for (auto& k : keys) if (const json::Value* x = d->find(k.first); x && x->t == json::Value::Num && x->n > 0) a.dims.push_back({k.second, x->n});
    }
    if (const json::Value* n = v.find("notes"); n && n->t == json::Value::Str) a.notes = n->s;
    a.ok = true;
    return a;
}

}  // namespace vision
}  // namespace dp

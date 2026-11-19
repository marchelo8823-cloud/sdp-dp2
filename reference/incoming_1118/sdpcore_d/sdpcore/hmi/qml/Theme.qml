// Цвета и шрифты пульта. Тёмная гамма — как у пультов СДП на мостике (ночью не слепит).
pragma Singleton
import QtQuick

QtObject {
    readonly property color bg: "#161a1f"
    readonly property color panel: "#1f252c"
    readonly property color panel2: "#262d35"
    readonly property color line: "#36404a"
    readonly property color text: "#dde4ea"
    readonly property color dim: "#8a96a3"
    readonly property color accent: "#4ba3de"      // заданное: точка, курс, команды
    readonly property color vessel: "#7cc2ee"
    readonly property color ok: "#4cb874"
    readonly property color warn: "#e3b23c"
    readonly property color alarm: "#e5534b"
    readonly property color info: "#8fa7bd"
    readonly property color fb: "#e8e8e8"          // факт (обратная связь)
    readonly property color wind: "#c9a6f2"
    readonly property color dist: "#f29e6b"        // внешнее возмущение
    readonly property string font: "DejaVu Sans"
    readonly property string mono: "DejaVu Sans Mono"
    readonly property int fs: 15
    readonly property int fsSmall: 13
    readonly property int fsBig: 30

    function num(v, d) {           // число по-русски: запятая
        if (v === undefined || v === null || isNaN(v)) return "—"
        return Number(v).toLocaleString(Qt.locale("ru_RU"), 'f', d === undefined ? 1 : d)
    }
    function deg(v) { return num(v, 1) + "°" }
    function knots(ms) { return ms * 1.943844 }
    function stateColor(code) { return code === 3 ? ok : code === 2 ? accent : code === 1 ? warn : dim }
    function levelColor(l) { return l === 1 ? alarm : l === 2 ? warn : info }
}

// Плитка движителя: шкала угла (у поворотных) — команда пунктиром, факт сплошной; полоса упора; числа команды и факта.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "."

Rectangle {
    id: tile
    property var t: ({})
    color: Theme.panel2
    radius: 3
    border.color: t.failed ? Theme.alarm : (t.taken ? Theme.line : Theme.panel2)
    border.width: t.failed ? 2 : 1
    implicitHeight: 196

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 8; spacing: 4
        RowLayout {
            Layout.fillWidth: true
            Label { text: t.id || ""; color: Theme.text; font.bold: true; font.pixelSize: 17 }
            Label { text: t.type || ""; color: Theme.dim; font.pixelSize: Theme.fsSmall; Layout.fillWidth: true }
            Rectangle {
                radius: 2; implicitWidth: st.implicitWidth + 10; implicitHeight: st.implicitHeight + 4
                color: t.failed ? Theme.alarm : t.taken ? Qt.darker(Theme.ok, 1.6) : Theme.line
                Label { id: st; anchors.centerIn: parent; font.pixelSize: 12; color: Theme.text
                        text: t.failed ? "ОТКАЗ" : !t.enabled ? "вне работы" : t.taken ? "в управлении" : "не взят" }
            }
        }
        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 10
            Canvas {
                id: dial
                Layout.preferredWidth: 110; Layout.preferredHeight: 110
                onPaint: {
                    const g = getContext("2d"); g.reset()
                    const c = width / 2, r = width / 2 - 6
                    g.strokeStyle = Theme.line; g.lineWidth = 2; g.beginPath(); g.arc(c, c, r, 0, 2 * Math.PI); g.stroke()
                    g.fillStyle = Theme.dim; g.font = "11px '" + Theme.font + "'"; g.textAlign = "center"; g.fillText("нос", c, 13)
                    const ray = function (deg, col, dash) {
                        const a = deg * Math.PI / 180
                        g.strokeStyle = col; g.lineWidth = dash ? 2 : 4; g.setLineDash(dash ? [5, 4] : [])
                        g.beginPath(); g.moveTo(c, c); g.lineTo(c + Math.sin(a) * (r - 4), c - Math.cos(a) * (r - 4)); g.stroke(); g.setLineDash([])
                    }
                    if (t.azimuth) { ray(t.cmdAng, Theme.accent, true); if (t.fb) ray(t.fbAng, Theme.fb, false) }
                    else {   // туннельный: стрелка на борт по знаку упора
                        const p = t.fb ? t.fbPct : t.cmdPct
                        if (Math.abs(p) > 1) ray(p > 0 ? 90 : 270, Theme.fb, false)
                        g.fillStyle = Theme.dim; g.fillText("туннель", c, c + r * 0.55)
                    }
                    g.fillStyle = Theme.text; g.beginPath(); g.arc(c, c, 4, 0, 2 * Math.PI); g.fill()
                }
                Connections { target: tile; function onTChanged() { dial.requestPaint() } }
            }
            // полоса упора: −100…+100 %, команда — рамка, факт — заливка
            Item {
                Layout.preferredWidth: 22; Layout.fillHeight: true
                Rectangle { anchors.fill: parent; color: Theme.bg; border.color: Theme.line }
                Rectangle { x: 0; width: parent.width; height: 1; y: parent.height / 2; color: Theme.dim }
                Rectangle {
                    property real p: Math.max(-100, Math.min(100, t.fb ? t.fbPct : t.cmdPct)) / 100
                    x: 2; width: parent.width - 4
                    height: Math.abs(p) * (parent.height / 2 - 2)
                    y: p >= 0 ? parent.height / 2 - height : parent.height / 2
                    color: Math.abs(p) > 0.9 ? Theme.warn : Theme.fb
                }
                Rectangle {
                    property real p: Math.max(-100, Math.min(100, t.cmdPct || 0)) / 100
                    x: -3; width: parent.width + 6; height: 2; color: Theme.accent
                    y: parent.height / 2 - p * (parent.height / 2 - 2)
                }
            }
            GridLayout {
                columns: 2; rowSpacing: 2; columnSpacing: 8; Layout.fillWidth: true
                Label { text: "упор"; color: Theme.dim; font.pixelSize: Theme.fsSmall }
                Label { text: Theme.num(t.thrustKN, 0) + " кН"; color: Theme.text; font.pixelSize: Theme.fs; font.family: Theme.mono }
                Label { text: "команда"; color: Theme.dim; font.pixelSize: Theme.fsSmall }
                Label { text: Theme.num(t.cmdPct, 0) + " %"; color: Theme.accent; font.pixelSize: Theme.fs; font.family: Theme.mono }
                Label { text: "факт"; color: Theme.dim; font.pixelSize: Theme.fsSmall }
                Label { text: t.fb ? Theme.num(t.fbPct, 0) + " %" : "—"; color: Theme.fb; font.pixelSize: Theme.fs; font.family: Theme.mono }
                Label { text: "угол"; color: Theme.dim; font.pixelSize: Theme.fsSmall; visible: t.azimuth === true }
                Label { visible: t.azimuth === true; font.pixelSize: Theme.fs; font.family: Theme.mono; color: Theme.text
                        text: Theme.num(t.cmdAng, 0) + "° / " + (t.fb ? Theme.num(t.fbAng, 0) + "°" : "—") }
            }
        }
    }
}

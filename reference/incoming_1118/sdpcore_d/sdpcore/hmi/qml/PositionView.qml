// План положения: север вверх, в центре — заданная точка. Судно по оценке ядра, след, точка и курс задания,
// движители с векторами упора, ветер и внешнее возмущение у края. Масштаб — радиус внешнего кольца.
import QtQuick
import QtQuick.Controls
import "."

Rectangle {
    id: root
    color: Theme.panel
    property var s: ({})
    property var hull: ({})
    property var thrusters: []
    property var track: []
    property real range: 50          // м — радиус внешнего кольца
    property bool followVessel: false

    Canvas {
        id: cv
        anchors.fill: parent
        anchors.margins: 6
        onPaint: {
            const g = getContext("2d")
            const W = width, H = height
            g.reset()
            g.fillStyle = Theme.panel; g.fillRect(0, 0, W, H)
            const s = root.s
            if (!s.hasShip) {
                g.fillStyle = Theme.dim; g.font = "20px '" + Theme.font + "'"; g.textAlign = "center"
                g.fillText("Судно не загружено — «Открыть судно…»", W / 2, H / 2)
                return
            }
            const R = Math.min(W, H) / 2 - 34
            const k = R / root.range                 // пикселей на метр
            const cx = W / 2, cy = H / 2
            const c0n = root.followVessel ? s.n : s.refN, c0e = root.followVessel ? s.e : s.refE
            const X = function (e) { return cx + (e - c0e) * k }
            const Y = function (n) { return cy - (n - c0n) * k }
            // кольца дальности
            g.strokeStyle = Theme.line; g.lineWidth = 1; g.fillStyle = Theme.dim; g.font = "13px '" + Theme.font + "'"; g.textAlign = "left"
            for (let i = 1; i <= 4; ++i) {
                const r = R * i / 4
                g.beginPath(); g.arc(cx, cy, r, 0, 2 * Math.PI); g.stroke()
                g.fillText(Theme.num(root.range * i / 4, root.range >= 40 ? 0 : 1) + " м", cx + r * 0.707 + 3, cy - r * 0.707 - 3)
            }
            g.beginPath(); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.stroke()
            // север
            g.fillStyle = Theme.text; g.font = "bold 16px '" + Theme.font + "'"; g.textAlign = "center"
            g.fillText("С", cx, cy - R - 12)
            // след
            const t = root.track
            if (t.length >= 4) {
                g.strokeStyle = Qt.rgba(0.49, 0.76, 0.93, 0.45); g.lineWidth = 1.5; g.beginPath()
                g.moveTo(X(t[1]), Y(t[0]))
                for (let i = 2; i + 1 < t.length; i += 2) g.lineTo(X(t[i + 1]), Y(t[i]))
                g.stroke()
            }
            // заданная точка и курс задания
            const rx = X(s.refE), ry = Y(s.refN)
            g.strokeStyle = Theme.accent; g.lineWidth = 2
            g.beginPath(); g.moveTo(rx - 12, ry); g.lineTo(rx + 12, ry); g.moveTo(rx, ry - 12); g.lineTo(rx, ry + 12); g.stroke()
            g.beginPath(); g.arc(rx, ry, 6, 0, 2 * Math.PI); g.stroke()
            const rh = s.refHdg * Math.PI / 180
            g.setLineDash([6, 5]); g.beginPath(); g.moveTo(rx, ry); g.lineTo(rx + Math.sin(rh) * R * 0.35, ry - Math.cos(rh) * R * 0.35); g.stroke(); g.setLineDash([])
            // судно
            const L = hull.length || 60, B = hull.beam || 14
            const h = s.hdg * Math.PI / 180, ch = Math.cos(h), sh = Math.sin(h)
            const P = function (x, y) { return [X(s.e + x * sh + y * ch), Y(s.n + x * ch - y * sh)] }   // x — к носу, y — на правый борт
            const shape = [[L / 2, 0], [L / 2 - B * 0.9, B / 2], [-L / 2, B / 2], [-L / 2, -B / 2], [L / 2 - B * 0.9, -B / 2]]
            g.beginPath()
            for (let i = 0; i < shape.length; ++i) { const p = P(shape[i][0], shape[i][1]); if (i === 0) g.moveTo(p[0], p[1]); else g.lineTo(p[0], p[1]) }
            g.closePath()
            g.fillStyle = s.posValid ? Qt.rgba(0.49, 0.76, 0.93, 0.22) : Qt.rgba(0.9, 0.33, 0.29, 0.25)
            g.fill(); g.strokeStyle = s.posDR ? Theme.warn : Theme.vessel; g.lineWidth = 2; g.stroke()
            // курсовая черта
            const bow = P(L / 2, 0), far = P(L / 2 + L * 0.6, 0)
            g.strokeStyle = Theme.vessel; g.lineWidth = 1.5; g.beginPath(); g.moveTo(bow[0], bow[1]); g.lineTo(far[0], far[1]); g.stroke()
            // движители: вектор упора (факт), длина ∝ доле упора
            const th = root.thrusters
            for (let i = 0; i < th.length; ++i) {
                const q = th[i], p = P(q.x, q.y)
                g.fillStyle = q.failed ? Theme.alarm : q.taken ? Theme.ok : Theme.dim
                g.beginPath(); g.arc(p[0], p[1], 4, 0, 2 * Math.PI); g.fill()
                const pct = q.fb ? q.fbPct : q.cmdPct
                if (Math.abs(pct) > 1) {
                    const a = (q.azimuth ? q.fbAng : 90) * Math.PI / 180 + h   // направление силы на судно
                    const len = Math.min(1, Math.abs(pct) / 100) * 46 * (pct < 0 ? -1 : 1)   // пикселей: до 46 при 100 %
                    g.strokeStyle = Theme.fb; g.lineWidth = 2; g.beginPath(); g.moveTo(p[0], p[1])
                    g.lineTo(p[0] + Math.sin(a) * len, p[1] - Math.cos(a) * len); g.stroke()
                }
            }
            // истинное положение в имитаторе (для проверки оценки)
            if (s.sim && s.trueN !== undefined) {
                g.strokeStyle = Theme.dim; g.lineWidth = 1; const tx = X(s.trueE), ty = Y(s.trueN)
                g.beginPath(); g.moveTo(tx - 5, ty - 5); g.lineTo(tx + 5, ty + 5); g.moveTo(tx - 5, ty + 5); g.lineTo(tx + 5, ty - 5); g.stroke()
            }
            // ветер (откуда, относительно носа → истинное) и внешнее возмущение (куда) — стрелки у края
            const arrow = function (angRad, col, label, inward) {
                const ex = cx + Math.sin(angRad) * (R + 18), ey = cy - Math.cos(angRad) * (R + 18)
                const ix = cx + Math.sin(angRad) * (R - 22), iy = cy - Math.cos(angRad) * (R - 22)
                const fx = inward ? ex : ix, fy = inward ? ey : iy, tx2 = inward ? ix : ex, ty2 = inward ? iy : ey
                g.strokeStyle = col; g.fillStyle = col; g.lineWidth = 3
                g.beginPath(); g.moveTo(fx, fy); g.lineTo(tx2, ty2); g.stroke()
                const a2 = Math.atan2(ty2 - fy, tx2 - fx)
                g.beginPath(); g.moveTo(tx2, ty2); g.lineTo(tx2 - 12 * Math.cos(a2 - 0.4), ty2 - 12 * Math.sin(a2 - 0.4)); g.lineTo(tx2 - 12 * Math.cos(a2 + 0.4), ty2 - 12 * Math.sin(a2 + 0.4)); g.closePath(); g.fill()
                g.font = "13px '" + Theme.font + "'"; g.textAlign = "center"; g.fillText(label, ex + Math.sin(angRad) * 12, ey - Math.cos(angRad) * 12)
            }
            if (s.windValid) arrow((s.windFromRel + s.hdg) * Math.PI / 180, Theme.wind, "ветер", true)
            const dmag = Math.sqrt(s.distN * s.distN + s.distE * s.distE)
            if (dmag > 2000) arrow(Math.atan2(s.distE, s.distN), Theme.dist, "возмущение", false)
        }
    }
    onSChanged: cv.requestPaint()
    onRangeChanged: cv.requestPaint()
    onFollowVesselChanged: cv.requestPaint()

    Row {
        anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 10
        spacing: 4
        Label { text: "Масштаб:"; color: Theme.dim; font.pixelSize: Theme.fsSmall; anchors.verticalCenter: parent.verticalCenter }
        Repeater {
            model: [10, 25, 50, 100, 250]
            delegate: Button {
                text: modelData + " м"; flat: true; checkable: true; checked: root.range === modelData
                font.pixelSize: Theme.fsSmall; implicitWidth: 62
                onClicked: root.range = modelData
            }
        }
        CheckBox { text: "За судном"; checked: root.followVessel; onToggled: root.followVessel = checked; font.pixelSize: Theme.fsSmall }
    }
    Column {
        anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.margins: 12
        spacing: 2
        Repeater {
            model: [["＋", Theme.accent, "заданная точка"], ["—", Theme.vessel, "судно (оценка ядра)"], ["×", Theme.dim, "истинное положение (имитатор)"], ["→", Theme.wind, "ветер, откуда"], ["→", Theme.dist, "внешнее возмущение, куда"]]
            delegate: Row {
                spacing: 6
                Label { text: modelData[0]; color: modelData[1]; font.pixelSize: Theme.fsSmall; width: 14 }
                Label { text: modelData[2]; color: Theme.dim; font.pixelSize: Theme.fsSmall }
            }
        }
    }
}

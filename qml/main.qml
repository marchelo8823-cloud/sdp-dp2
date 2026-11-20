// SDP-DP2 — QML-HMI пульта оператора (стилистика Kongsberg K-Pos).
// Данные: контекстное свойство telem (QObject):
//   Telem.state — QVariantMap телеметрии (сглаженная, 20 Гц)
//   Telem.tick() — сигнал перерисовки
//   Telem.cmd({...}) — слот команд (UDP в sim_server)
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtMultimedia
import Sdp 1.0

ApplicationWindow {
    id: root
    objectName: "sdpHmi"
    visible: true
    width: 1680
    height: 1000
    minimumWidth: 1280
    minimumHeight: 780
    title: "SDP-DP2 — Пульт оператора ДП"

    // ---------------------------------------------------------- тема
    property bool dark: true
    readonly property var pal: dark ? ({
        "bg": "#0a0f1a", "panel": "#111a2c", "panel2": "#0d1524",
        "line": "#1e2c47", "txt": "#c9d6ea", "dim": "#5f7396",
        "disk1": "#101a30", "disk2": "#0a1120"
    }) : ({
        "bg": "#dfe5ee", "panel": "#f4f7fb", "panel2": "#e9eef6",
        "line": "#c3cede", "txt": "#1b2a41", "dim": "#5b6b84",
        "disk1": "#eef2f9", "disk2": "#dde5f0"
    })
    readonly property color green: "#2fd07c"
    readonly property color yellow: "#ffc542"
    readonly property color red: "#ff5252"
    readonly property color cyan: "#39c2d7"
    readonly property color blue: "#4d8dff"
    readonly property color orange: "#ff9f43"

    color: pal.bg

    // ---------------------------------------------------------- состояние UI
    property real range: 100
    property var trail: []
    property var trend: []
    property real thrustSmooth: -1
    property real lastTrailT: -1
    // имитатор заблокирован по умолчанию (РС 8.5.7: команды симулятора
    // на рабочем пульте — потенциально одно непреднамеренное действие);
    // доступ — удержание замка 2 с
    property bool instrLock: true
    property real lockProg: 0
    // звуковая сигнализация: «тихо» отдельно от квитирования (MG 6.1.10);
    // новая авария снова включает звук
    property bool sndMuted: false
    property int lastAlarmId: 0

    SoundEffect {
        id: warnSnd
        source: "snd/warn.wav"
        loops: SoundEffect.Infinite
        volume: 0.5
    }
    SoundEffect {
        id: alarmSnd
        source: "snd/alarm.wav"
        loops: SoundEffect.Infinite
        volume: 0.7
    }
    function updateSound() {
        var s = Telem.state;
        if (s === undefined || s.t === undefined) {
            if (warnSnd.playing) warnSnd.stop();
            if (alarmSnd.playing) alarmSnd.stop();
            return;
        }
        var maxId = 0;
        if (s.alarms)
            for (var ai = 0; ai < s.alarms.length; ai++)
                if (!s.alarms[ai].acked && s.alarms[ai].id > maxId)
                    maxId = s.alarms[ai].id;
        if (maxId > root.lastAlarmId)
            root.sndMuted = false;      // новая авария — звук снова
        root.lastAlarmId = maxId;
        var worst = s.alarm_worst || 0;
        var needAlarm = !root.sndMuted && worst >= 2;
        var needWarn = !root.sndMuted && worst === 1;
        if (needAlarm && !alarmSnd.playing) alarmSnd.play();
        if (!needAlarm && alarmSnd.playing) alarmSnd.stop();
        if (needWarn && !warnSnd.playing) warnSnd.play();
        if (!needWarn && warnSnd.playing) warnSnd.stop();
    }

    readonly property var shipN: [38, 12, -30, -38, -38, -30, 12]
    readonly property var shipE: [0, 9, 9, 4, -4, -9, -9]
    readonly property var thrPos: [[30, 0], [18, 0], [-30, 6], [-30, -6]]

    // --- подтверждение уставок (Mega-Guard 6.1.3–6.1.4: Apply/Cancel;
    // РС 8.5.7 — одно непреднамеренное действие не теряет позицию)
    property bool pendPos: false
    property real pendN: 0
    property real pendE: 0
    property bool pendHdg: false
    property real pendHdgDeg: 0

    function jogSetpoint(dn, de) {
        if (!hasData()) return;
        if (!pendPos) {
            pendN = Telem.state.eta_d[0];
            pendE = Telem.state.eta_d[1];
            pendPos = true;
        }
        pendN += dn;
        pendE += de;
    }
    function setPendingPoint(n, e) {
        pendN = n; pendE = e; pendPos = true;
    }
    function bumpPendingHdg(d) {
        if (!hasData()) return;
        if (!pendHdg) {
            pendHdgDeg = Telem.state.hdg_d_deg;
            pendHdg = true;
        }
        pendHdgDeg = (pendHdgDeg + d + 360) % 360;
    }
    function applyPending() {
        if (pendPos)
            Telem.cmd({ "action": "setpoint",
                        "north": pendN, "east": pendE });
        if (pendHdg)
            Telem.cmd({ "action": "heading", "deg": pendHdgDeg });
        pendPos = false;
        pendHdg = false;
    }
    function cancelPending() {
        pendPos = false;
        pendHdg = false;
    }

    function hasData() {
        var s = Telem.state;
        return s !== undefined && s.t !== undefined;
    }

    readonly property int statusLevel: {
        var s = Telem.state;
        if (s === undefined || s.t === undefined) return 3;
        if (s.degrad !== undefined) {
            if (s.degrad.thrust_released || s.degrad.hdg_lost) return 2;
            if (s.degrad.pos_lost) return 1;
        }
        if (s.alarm_worst >= 2 || s.conseq.level >= 2) return 2;
        if (s.alarm_worst === 1 || s.conseq.level === 1 ||
                s.pos_err > s.warn_pos) return 1;
        return 0;
    }

    Connections {
        target: Telem
        function onTick() {
            var s = Telem.state;
            root.updateSound();
            if (s === undefined || s.t === undefined) return;
            if (s.t - root.lastTrailT > 1.0) {
                if (root.lastTrailT >= 0 && s.t < root.lastTrailT)
                    root.trail = [];
                root.trail.push([s.err_n, s.err_e]);
                if (root.trail.length > 600) root.trail.shift();
                root.lastTrailT = s.t;
            }
            var nr = Math.min(500, Math.max(30, 1.5 * s.pos_err,
                                            1.7 * s.alarm_pos));
            if (Math.abs(nr - root.range) / root.range > 0.15)
                root.range = nr;
            var thrust = Math.abs(s.resultant.force_kn) * 10;
            if (root.thrustSmooth < 0) root.thrustSmooth = thrust;
            root.thrustSmooth += 0.03 * (thrust - root.thrustSmooth);
            root.trend.push([s.pos_err, s.wind_meas.speed_kn,
                             root.thrustSmooth]);
            if (root.trend.length > 2400) root.trend.shift();
            // джойстик: синхронизация ручки с состоянием сервера
            if (s.joy !== undefined && !joyPadArea.pressed) {
                joyPad.jx = s.joy[1];
                joyPad.jy = s.joy[0];
                joyPad.requestPaint();
            }
            posPlot.requestPaint();
            compass.requestPaint();
            windRose.requestPaint();
            mimic.requestPaint();
            trendsCanvas.requestPaint();
        }
    }

    // ---------------------------------------------------------- компоненты
    component DpPanel: Rectangle {
        id: panelRoot
        property string title: ""
        property alias rightTitle: rightLabel.text
        default property alias body: bodyItem.data
        color: root.pal.panel
        radius: 6
        border.color: root.pal.line
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 8
            spacing: 4
            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: panelRoot.title.toUpperCase()
                    color: root.pal.dim
                    font.pixelSize: 11
                    font.letterSpacing: 1
                }
                Item { Layout.fillWidth: true }
                Text {
                    id: rightLabel
                    color: root.green
                    font.pixelSize: 10
                    font.family: "Menlo"
                }
            }
            Item {
                id: bodyItem
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
    }

    component KV: RowLayout {
        property string k: ""
        property string v: "—"
        property color vColor: root.pal.txt
        spacing: 6
        Text { text: k; color: root.pal.dim; font.pixelSize: 12 }
        Item { Layout.fillWidth: true }
        Text {
            text: v
            color: vColor
            font.family: "Menlo"
            font.pixelSize: 12
            font.bold: true
        }
    }

    component LedRow: RowLayout {
        property string name: ""
        property bool failed: false
        spacing: 6
        Rectangle {
            width: 10; height: 10; radius: 5
            color: failed ? root.red : root.green
            Behavior on color { ColorAnimation { duration: 200 } }
        }
        Text { text: name; color: root.pal.txt; font.pixelSize: 12 }
        Item { Layout.fillWidth: true }
        Text {
            text: failed ? "ОТКАЗ" : "OK"
            color: failed ? root.red : root.pal.txt
            font.family: "Menlo"
            font.pixelSize: 12
            font.bold: true
        }
    }

    // строка опорной системы: состояние, вес/σ, снятие/возврат (как Refsys)
    component SysRow: RowLayout {
        property string name: ""
        property string st: "ok"        // ok | rejected | deselected | failed
        property string reason: ""
        property string info: ""
        property bool disagree: false
        property bool used: false
        property string kind: "prs"
        property int idx: 0
        spacing: 6
        Rectangle {
            width: 10; height: 10; radius: 5
            color: st === "ok" ? (disagree ? root.yellow : root.green)
                 : st === "deselected" ? root.pal.dim : root.red
        }
        Text { text: name; color: root.pal.txt; font.pixelSize: 12 }
        Text {
            visible: used
            text: "● принят"
            color: root.cyan; font.pixelSize: 10
        }
        Text {
            text: st === "ok" ? (disagree ? "РАСХОДИТСЯ" : "В РАБОТЕ")
                : st === "rejected" ? "ОТБРАК: " + reason
                : st === "deselected" ? "СНЯТА" : "ОТКАЗ"
            color: st === "ok" ? (disagree ? root.yellow : root.green)
                 : st === "deselected" ? root.pal.dim : root.red
            font.pixelSize: 10
            font.family: "Menlo"
            elide: Text.ElideRight
            Layout.maximumWidth: 140
        }
        Item { Layout.fillWidth: true }
        Text { text: info; color: root.pal.dim; font.pixelSize: 10; font.family: "Menlo" }
        // снятие системы — критичное действие, с подтверждением;
        // возврат — безопасное, сразу
        DpConfirmButton {
            visible: st === "ok"
            label: "Снять"
            confirmLabel: "Снять?"
            padding: 3
            onConfirmed: Telem.cmd({ "action": "sensor_deselect",
                                     "kind": kind, "idx": idx,
                                     "on": true })
        }
        DpButton {
            visible: st !== "ok" && st !== "failed"
            text: "Вернуть"
            padding: 3
            onClicked: {
                if (st === "deselected")
                    Telem.cmd({ "action": "sensor_deselect", "kind": kind,
                                "idx": idx, "on": false })
                else
                    Telem.cmd({ "action": "sensor_restore", "kind": kind,
                                "idx": idx })
            }
        }
    }

    component DpButton: Button {
        id: control
        property bool danger: false
        property bool accent: false
        property bool active: false
        padding: 6
        contentItem: Text {
            text: control.text
            color: control.danger ? root.red
                 : control.active ? root.cyan : root.pal.txt
            font.pixelSize: 12
            font.bold: control.active
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
        background: Rectangle {
            color: control.down ? root.pal.line
                 : control.active ? Qt.rgba(root.cyan.r, root.cyan.g,
                                            root.cyan.b, 0.15)
                 : root.pal.panel2
            radius: 4
            border.color: control.danger ? root.red
                        : control.active ? root.cyan
                        : control.accent ? root.orange : root.pal.line
        }
    }

    // Кнопка с двухэтапным подтверждением (РС 8.5.7): первое нажатие
    // взводит на 3 с («ПОДТВЕРДИ»), второе — исполняет. Как повторное
    // нажатие режимов на Mega-Guard / K-Pos.
    component DpConfirmButton: DpButton {
        id: cfb
        property string label: ""
        property string confirmLabel: "ПОДТВЕРДИ"
        property bool armed: false
        signal confirmed()
        text: armed ? confirmLabel : label
        accent: armed
        Timer {
            id: armTimer
            interval: 3000
            repeat: false
            onTriggered: cfb.armed = false
        }
        onClicked: {
            if (cfb.armed) {
                cfb.armed = false;
                armTimer.stop();
                cfb.confirmed();
            } else {
                cfb.armed = true;
                armTimer.restart();
            }
        }
    }

    // ---------------------------------------------------------- каркас
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 6

        // ---------------- header
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 42
            color: root.pal.panel
            radius: 6
            border.color: root.pal.line
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 14
                Text {
                    text: "SDP-DP2"
                    color: root.cyan
                    font.pixelSize: 16
                    font.bold: true
                    font.letterSpacing: 0.5
                }
                Rectangle {
                    color: root.hasData() && Telem.state.mode !== "auto"
                           ? root.orange : root.blue
                    radius: 3
                    implicitWidth: modeTxt.implicitWidth + 20
                    implicitHeight: 22
                    Text {
                        id: modeTxt
                        anchors.centerIn: parent
                        text: !root.hasData() ? "AUTO POSITION"
                            : Telem.state.mode === "man_joy" ? "MAN JOY"
                            : Telem.state.mode === "auto_joy" ? "AUTO JOY"
                            : "AUTO POSITION"
                        color: "white"
                        font.pixelSize: 11
                        font.bold: true
                    }
                }
                Text {
                    font.family: "Menlo"
                    font.pixelSize: 13
                    font.bold: true
                    color: root.pal.txt
                    text: {
                        var s = Telem.state;
                        if (s === undefined || s.t === undefined) return "—";
                        var mm = Math.floor(s.t / 60), ss = Math.floor(s.t % 60);
                        return "T+" + (mm < 10 ? "0" : "") + mm + ":"
                                + (ss < 10 ? "0" : "") + ss
                                + (s.paused ? " ⏸" : "")
                                + " · " + s.speed.toFixed(0) + "×";
                    }
                }
                // состав контура (замечание операторов: режим
                // регулятора должен быть виден)
                Rectangle {
                    radius: 3
                    implicitWidth: ctrlTxt.implicitWidth + 14
                    implicitHeight: 20
                    color: "transparent"
                    border.color: root.cyan
                    Text {
                        id: ctrlTxt
                        anchors.centerIn: parent
                        font.pixelSize: 10
                        font.bold: true
                        color: root.cyan
                        text: root.hasData() && Telem.state.ctrl
                              ? "РЕГ " + Telem.state.ctrl.law
                                + " · " + Telem.state.ctrl.est
                                + " · " + Telem.state.ctrl.alloc
                              : "РЕГ —"
                    }
                }
                Rectangle {
                    radius: 3
                    implicitWidth: caTxt.implicitWidth + 14
                    implicitHeight: 20
                    color: "transparent"
                    border.color: root.hasData() && Telem.state.ctrl
                                  && Telem.state.ctrl.ca_on
                                  ? root.green : root.red
                    Text {
                        id: caTxt
                        anchors.centerIn: parent
                        font.pixelSize: 10
                        font.bold: true
                        color: parent.border.color
                        text: root.hasData() && Telem.state.ctrl
                              && Telem.state.ctrl.ca_on
                              ? "CA ВКЛ" : "CA ВЫКЛ"
                    }
                }
                // voyage recorder — индикация записи
                Text {
                    id: recLamp
                    visible: root.hasData() && Telem.state.rec
                             && Telem.state.rec.on
                    text: "● REC"
                    color: root.red
                    font.pixelSize: 11
                    font.bold: true
                    SequentialAnimation on opacity {
                        running: recLamp.visible
                        loops: Animation.Infinite
                        NumberAnimation { to: 0.35; duration: 800 }
                        NumberAnimation { to: 1.0; duration: 800 }
                    }
                }
                Rectangle {
                    id: lamp
                    radius: 3
                    implicitWidth: lampTxt.implicitWidth + 20
                    implicitHeight: 22
                    color: "transparent"
                    border.color: root.statusLevel === 2 ? root.red
                                : root.statusLevel === 1 ? root.yellow
                                : root.statusLevel === 3 ? root.red : root.green
                    Text {
                        id: lampTxt
                        anchors.centerIn: parent
                        font.pixelSize: 12
                        font.bold: true
                        color: parent.border.color
                        text: root.statusLevel === 2 ? "DP STATUS: ALARM"
                            : root.statusLevel === 1 ? "DP STATUS: WARNING"
                            : root.statusLevel === 3 ? "НЕТ СВЯЗИ"
                            : "DP STATUS: NORMAL"
                    }
                    SequentialAnimation on opacity {
                        running: root.statusLevel >= 2
                        loops: Animation.Infinite
                        NumberAnimation { to: 0.45; duration: 500 }
                        NumberAnimation { to: 1.0; duration: 500 }
                        onStopped: lamp.opacity = 1.0
                    }
                }
                Item { Layout.fillWidth: true }
                Text {
                    Layout.maximumWidth: 460
                    elide: Text.ElideRight
                    font.pixelSize: 12
                    font.bold: true
                    color: root.statusLevel === 2 ? root.red
                         : (root.hasData() && Telem.state.conseq.level === 1)
                           ? root.yellow : root.green
                    text: root.hasData() ? Telem.state.conseq.text : "…"
                }
                Item { Layout.fillWidth: true }
                DpButton {
                    text: root.hasData() ? String(Telem.state.unacked) : "0"
                    background: Rectangle {
                        radius: 9
                        color: root.hasData() && Telem.state.unacked > 0
                               ? root.red : root.pal.line
                    }
                    contentItem: Text {
                        text: parent.text
                        color: root.hasData() && Telem.state.unacked > 0
                               ? "white" : root.pal.dim
                        font.bold: true
                        horizontalAlignment: Text.AlignHCenter
                    }
                    onClicked: Telem.cmd({ "action": "ack" })
                }
                DpButton {
                    // остановка звука — отдельно от квитирования (MG 6.1.10)
                    text: root.sndMuted ? "🔇" : "🔔"
                    active: !root.sndMuted && root.hasData()
                            && Telem.state.alarm_worst >= 1
                    onClicked: root.sndMuted = !root.sndMuted
                }
                DpButton {
                    text: "ACK"
                    onClicked: Telem.cmd({ "action": "ack" })
                }
                DpButton {
                    text: "☀/☾"
                    onClicked: root.dark = !root.dark
                }
            }
        }

        // ---------------- баннер деградации
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 46
            visible: root.hasData() && Telem.state.degrad.state !== "normal"
            radius: 6
            color: root.hasData() && Telem.state.degrad.state === "position_dr"
                   ? Qt.rgba(1, 0.77, 0.26, 0.10) : Qt.rgba(1, 0.32, 0.32, 0.12)
            border.color: root.hasData() &&
                          Telem.state.degrad.state === "position_dr"
                          ? root.yellow : root.red
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 12
                anchors.rightMargin: 12
                spacing: 14
                Text {
                    font.pixelSize: 13
                    font.bold: true
                    color: parent.parent.border.color
                    text: {
                        if (!root.hasData()) return "";
                        var d = Telem.state.degrad;
                        if (d.state === "heading_dr")
                            return "СЧИСЛЕНИЕ КУРСА — нет гирокомпасов "
                                + d.hdg_elapsed.toFixed(1) + " с · упор будет "
                                + "снят через " + d.release_in.toFixed(1) + " с";
                        if (d.state === "thrust_released")
                            return "УПОР СНЯТ — НЕТ КУРСА · команды "
                                + "движителям 0 %";
                        if (d.state === "position_dr")
                            return "УДЕРЖАНИЕ ПО МОДЕЛИ — нет систем "
                                + "положения " + d.pos_elapsed.toFixed(0)
                                + " с · вероятный уход до "
                                + d.excursion_m.toFixed(0) + " м";
                        return "";
                    }
                }
                Item { Layout.fillWidth: true }
                Text {
                    Layout.maximumWidth: 480
                    wrapMode: Text.Wrap
                    font.pixelSize: 11
                    color: root.pal.dim
                    text: {
                        if (!root.hasData()) return "";
                        var d = Telem.state.degrad;
                        if (d.state === "heading_dr")
                            return "ЧТО ДЕЛАТЬ: проверьте гирокомпасы "
                                + "(питание, связь). Когда курс вернётся — "
                                + "упор сам не включится.";
                        if (d.state === "thrust_released")
                            return "ЧТО ДЕЛАТЬ: управление вручную. "
                                + "Гирокомпасы вернулись — «Вернуть упор».";
                        if (d.state === "position_dr")
                            return "ЧТО ДЕЛАТЬ: проверьте опорные системы. "
                                + "Готовьтесь к ручному управлению.";
                        return "";
                    }
                }
                DpButton {
                    visible: root.hasData() &&
                             Telem.state.degrad.thrust_released
                    text: "Вернуть упор"
                    onClicked: Telem.cmd({ "action": "thrust_restore" })
                }
            }
        }

        // ---------------- основная сетка
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 6

            // ===== position plot
            DpPanel {
                title: "Position plot"
                rightTitle: "range " + Math.round(root.range) + " м"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 440
                Layout.minimumWidth: 320
                Canvas {
                    id: posPlot
                    anchors.fill: parent
                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.reset();
                        var w = width, h = height;
                        var R = Math.max(20, Math.min(w, h) / 2 - 14);
                        var cx = w / 2, cy = h / 2;
                        var s = Telem.state;
                        // диск
                        var g = ctx.createLinearGradient(cx, cy - R, cx, cy + R);
                        g.addColorStop(0, root.pal.disk1);
                        g.addColorStop(1, root.pal.disk2);
                        ctx.fillStyle = g;
                        ctx.strokeStyle = root.pal.line;
                        ctx.lineWidth = 1;
                        ctx.beginPath();
                        ctx.arc(cx, cy, R, 0, 2 * Math.PI);
                        ctx.fill();
                        ctx.stroke();
                        // кольца
                        ctx.setLineDash([4, 4]);
                        ctx.strokeStyle = root.pal.line;
                        for (var f = 1; f <= 3; f++) {
                            ctx.beginPath();
                            ctx.arc(cx, cy, R * f / 3, 0, 2 * Math.PI);
                            ctx.stroke();
                        }
                        ctx.setLineDash([]);
                        // риски и подписи
                        ctx.fillStyle = root.pal.dim;
                        ctx.strokeStyle = root.pal.dim;
                        ctx.font = Math.max(8, R / 30) + "px Menlo";
                        ctx.textAlign = "center";
                        for (var a = 0; a < 360; a += 10) {
                            var big = a % 30 === 0;
                            var r1 = R * (big ? 0.95 : 0.975);
                            var sa = Math.sin(a * Math.PI / 180);
                            var ca = Math.cos(a * Math.PI / 180);
                            ctx.beginPath();
                            ctx.moveTo(cx + r1 * sa, cy - r1 * ca);
                            ctx.lineTo(cx + R * sa, cy - R * ca);
                            ctx.stroke();
                            if (big)
                                ctx.fillText(String(a), cx + 0.86 * R * sa,
                                             cy - 0.86 * R * ca + 4);
                        }
                        ctx.textAlign = "left";
                        for (f = 1; f <= 3; f++)
                            ctx.fillText(String(Math.round(root.range * f / 3)),
                                         cx + 3, cy - R * f / 3 - 3);
                        ctx.fillStyle = root.red;
                        ctx.font = "bold " + Math.max(10, R / 24) + "px Menlo";
                        ctx.fillText("N", cx - 5, cy - R - 4);
                        if (!root.hasData()) return;
                        // watch circles
                        var rings = [[s.warn_pos, root.yellow],
                                     [s.alarm_pos, root.red]];
                        for (var ri = 0; ri < 2; ri++) {
                            ctx.setLineDash([5, 4]);
                            ctx.strokeStyle = rings[ri][1];
                            ctx.lineWidth = 1.5;
                            ctx.beginPath();
                            ctx.arc(cx, cy, rings[ri][0] / root.range * R,
                                    0, 2 * Math.PI);
                            ctx.stroke();
                        }
                        ctx.setLineDash([]);
                        // уставка: перекрестие
                        ctx.strokeStyle = root.red;
                        ctx.lineWidth = 1.6;
                        ctx.beginPath();
                        ctx.moveTo(cx - 8, cy); ctx.lineTo(cx + 8, cy);
                        ctx.moveTo(cx, cy - 8); ctx.lineTo(cx, cy + 8);
                        ctx.stroke();
                        // предлагаемая точка (до «Применить») — жёлтая
                        if (root.pendPos) {
                            var dn = root.pendN - s.eta_d[0];
                            var de = root.pendE - s.eta_d[1];
                            var mx = cx + de / root.range * R;
                            var my = cy - dn / root.range * R;
                            ctx.setLineDash([4, 3]);
                            ctx.strokeStyle = root.yellow;
                            ctx.lineWidth = 1.6;
                            ctx.beginPath();
                            ctx.moveTo(mx - 8, my); ctx.lineTo(mx + 8, my);
                            ctx.moveTo(mx, my - 8); ctx.lineTo(mx, my + 8);
                            ctx.stroke();
                            // пунктир от текущей уставки к предлагаемой
                            ctx.beginPath();
                            ctx.moveTo(cx, cy);
                            ctx.lineTo(mx, my);
                            ctx.stroke();
                            ctx.setLineDash([]);
                        }
                        // трасса с затуханием
                        var tr = root.trail;
                        if (tr.length > 1) {
                            var step = 10;
                            for (var i0 = 0; i0 < tr.length - 1; i0 += step) {
                                var i1 = Math.min(i0 + step, tr.length - 1);
                                var al = 0.15 + 0.75 * (i0 / tr.length);
                                ctx.strokeStyle = Qt.rgba(0.30, 0.55, 1.0, al);
                                ctx.lineWidth = 1.4;
                                ctx.beginPath();
                                var p0 = tr[i0];
                                ctx.moveTo(cx + p0[1] / root.range * R,
                                           cy - p0[0] / root.range * R);
                                for (var i = i0 + 1; i <= i1; i++) {
                                    var pt = tr[i];
                                    ctx.lineTo(cx + pt[1] / root.range * R,
                                               cy - pt[0] / root.range * R);
                                }
                                ctx.stroke();
                            }
                        }
                        // метки опорных систем: где каждая PRS видит
                        // судно относительно заданной точки (Refsys view)
                        if (s.sensors && s.sensors.prs_detail) {
                            ctx.font = Math.max(8, R / 26) + "px Menlo";
                            ctx.textAlign = "left";
                            for (var pi = 0; pi < s.sensors.prs_detail.length; pi++) {
                                var pr = s.sensors.prs_detail[pi];
                                if (pr.dn === null || pr.de === null) continue;
                                if (pr.state === "failed" || pr.state === "deselected")
                                    continue;
                                var pdn = pr.dn / root.range * R;
                                var pde = pr.de / root.range * R;
                                if (Math.hypot(pdn, pde) > R * 1.05) continue;
                                var px2 = cx + pde, py2 = cy - pdn;
                                var pcol = pr.state !== "ok" ? root.red
                                         : pr.disagree ? root.yellow
                                         : root.green;
                                ctx.beginPath();
                                ctx.arc(px2, py2, 3.5, 0, 2 * Math.PI);
                                ctx.fillStyle = pcol;
                                ctx.fill();
                                ctx.strokeStyle = root.pal.bg;
                                ctx.lineWidth = 1;
                                ctx.stroke();
                                ctx.fillStyle = pcol;
                                ctx.fillText("P" + (pi + 1), px2 + 6, py2 - 4);
                            }
                        }
                        // судно
                        ctx.save();
                        ctx.translate(cx + s.err_e / root.range * R,
                                      cy - s.err_n / root.range * R);
                        ctx.rotate(s.hdg_deg * Math.PI / 180);
                        var sc = R / root.range;
                        ctx.beginPath();
                        for (var j = 0; j < root.shipN.length; j++) {
                            var px = root.shipE[j] * sc;
                            var py = -root.shipN[j] * sc;
                            if (j === 0) ctx.moveTo(px, py);
                            else ctx.lineTo(px, py);
                        }
                        ctx.closePath();
                        ctx.shadowColor = root.blue;
                        ctx.shadowBlur = 14;
                        ctx.fillStyle = root.blue;
                        ctx.fill();
                        ctx.shadowBlur = 0;
                        ctx.strokeStyle = root.pal.bg;
                        ctx.lineWidth = 1;
                        ctx.stroke();
                        ctx.restore();
                    }
                    MouseArea {
                        anchors.fill: parent
                        // клик по плану — тоже только в «предлагаемую»
                        // точку: исполнение после «Применить» (РС 8.5.7)
                        onClicked: (mouse) => {
                            if (!root.hasData()) return;
                            var s = Telem.state;
                            var R = Math.min(width, height) / 2 - 14;
                            var e = (mouse.x - width / 2) / R * root.range;
                            var n = -(mouse.y - height / 2) / R * root.range;
                            root.setPendingPoint(s.eta_d[0] + n,
                                                 s.eta_d[1] + e);
                        }
                    }
                }
            }

            // ===== heading
            DpPanel {
                title: "Heading"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 290
                Layout.minimumWidth: 240
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 2
                    Canvas {
                        id: compass
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 170
                        onPaint: {
                            var ctx = getContext("2d");
                            ctx.reset();
                            var w = width, h = height;
                            var R = Math.max(20, Math.min(w, h) / 2 - 10);
                            var cx = w / 2, cy = h / 2;
                            var g = ctx.createLinearGradient(cx, cy - R, cx, cy + R);
                            g.addColorStop(0, root.pal.disk1);
                            g.addColorStop(1, root.pal.disk2);
                            ctx.fillStyle = g;
                            ctx.strokeStyle = root.pal.line;
                            ctx.lineWidth = 1.5;
                            ctx.beginPath();
                            ctx.arc(cx, cy, R, 0, 2 * Math.PI);
                            ctx.fill();
                            ctx.stroke();
                            for (var a = 0; a < 360; a += 5) {
                                var big = a % 30 === 0;
                                var r1 = R * (big ? 0.86 : 0.93);
                                var sa = Math.sin(a * Math.PI / 180);
                                var ca = Math.cos(a * Math.PI / 180);
                                ctx.strokeStyle = big ? root.pal.txt : root.pal.dim;
                                ctx.lineWidth = big ? 2 : 1;
                                ctx.beginPath();
                                ctx.moveTo(cx + r1 * sa, cy - r1 * ca);
                                ctx.lineTo(cx + 0.99 * R * sa,
                                           cy - 0.99 * R * ca);
                                ctx.stroke();
                                if (a % 90 === 0) {
                                    ctx.fillStyle = a === 0 ? root.red
                                                            : root.pal.txt;
                                    ctx.font = "bold " + Math.max(11, R / 8)
                                               + "px Menlo";
                                    ctx.textAlign = "center";
                                    ctx.fillText("NESW"[a / 90],
                                                 cx + 0.68 * R * sa,
                                                 cy - 0.68 * R * ca + 5);
                                }
                            }
                            if (!root.hasData()) return;
                            var s = Telem.state;
                            // жучок уставки
                            ctx.save();
                            ctx.translate(cx, cy);
                            ctx.rotate(s.hdg_d_deg * Math.PI / 180);
                            ctx.beginPath();
                            ctx.moveTo(0, -R - 2);
                            ctx.lineTo(7, -R + 13);
                            ctx.lineTo(-7, -R + 13);
                            ctx.closePath();
                            ctx.shadowColor = root.green;
                            ctx.shadowBlur = 10;
                            ctx.fillStyle = root.green;
                            ctx.fill();
                            ctx.restore();
                            // судно
                            ctx.save();
                            ctx.translate(cx, cy);
                            ctx.rotate(s.hdg_deg * Math.PI / 180);
                            var r6 = R * 0.62;
                            ctx.beginPath();
                            ctx.moveTo(0, -r6);
                            ctx.lineTo(r6 * 0.26, r6 * 0.68);
                            ctx.lineTo(0, r6 * 0.4);
                            ctx.lineTo(-r6 * 0.26, r6 * 0.68);
                            ctx.closePath();
                            ctx.shadowColor = root.blue;
                            ctx.shadowBlur = 12;
                            ctx.fillStyle = root.blue;
                            ctx.fill();
                            ctx.restore();
                            ctx.shadowBlur = 0;
                            ctx.fillStyle = root.pal.txt;
                            ctx.font = "bold " + Math.max(16, R / 5.5)
                                       + "px Menlo";
                            ctx.textAlign = "center";
                            ctx.fillText(s.hdg_deg.toFixed(1) + "°",
                                         cx, cy + 8);
                        }
                    }
                    KV { k: "ROT"; v: root.hasData() ? Telem.state.rot_dpm.toFixed(1) + " °/min" : "—" }
                    KV { k: "Отклонение курса"; v: root.hasData() ? Telem.state.hdg_err_deg.toFixed(1) + "°" : "—" }
                    KV { k: "SOG"; v: root.hasData() ? Telem.state.sog_kn.toFixed(2) + " kn" : "—" }
                    Text {
                        text: "ТОЧКА (шаг 1 м)"
                        color: root.pal.dim
                        font.pixelSize: 10
                        font.letterSpacing: 1
                        topPadding: 4
                    }
                    RowLayout {
                        spacing: 4
                        DpButton { text: "N"; Layout.fillWidth: true; onClicked: root.jogSetpoint(1, 0) }
                        DpButton { text: "S"; Layout.fillWidth: true; onClicked: root.jogSetpoint(-1, 0) }
                        DpButton { text: "W"; Layout.fillWidth: true; onClicked: root.jogSetpoint(0, -1) }
                        DpButton { text: "E"; Layout.fillWidth: true; onClicked: root.jogSetpoint(0, 1) }
                    }
                    Text {
                        text: "КУРС"
                        color: root.pal.dim
                        font.pixelSize: 10
                        font.letterSpacing: 1
                    }
                    RowLayout {
                        spacing: 4
                        DpButton { text: "−10°"; Layout.fillWidth: true; onClicked: root.bumpPendingHdg(-10) }
                        DpButton { text: "−1°"; Layout.fillWidth: true; onClicked: root.bumpPendingHdg(-1) }
                        DpButton { text: "+1°"; Layout.fillWidth: true; onClicked: root.bumpPendingHdg(1) }
                        DpButton { text: "+10°"; Layout.fillWidth: true; onClicked: root.bumpPendingHdg(10) }
                    }
                    // панель подтверждения новой уставки (MG Apply/Cancel)
                    Rectangle {
                        visible: root.pendPos || root.pendHdg
                        Layout.fillWidth: true
                        implicitHeight: pendCol.implicitHeight + 10
                        radius: 4
                        color: Qt.rgba(root.yellow.r, root.yellow.g,
                                       root.yellow.b, 0.10)
                        border.color: root.yellow
                        ColumnLayout {
                            id: pendCol
                            anchors.fill: parent
                            anchors.margins: 5
                            spacing: 3
                            Text {
                                visible: root.pendPos
                                text: "Новая точка: N "
                                      + root.pendN.toFixed(1) + "  E "
                                      + root.pendE.toFixed(1)
                                color: root.yellow
                                font.pixelSize: 11
                                font.family: "Menlo"
                            }
                            Text {
                                visible: root.pendHdg
                                text: "Новый курс: "
                                      + root.pendHdgDeg.toFixed(0) + "°"
                                color: root.yellow
                                font.pixelSize: 11
                                font.family: "Menlo"
                            }
                            RowLayout {
                                spacing: 4
                                DpButton {
                                    text: "Применить"
                                    Layout.fillWidth: true
                                    active: true
                                    onClicked: root.applyPending()
                                }
                                DpButton {
                                    text: "Отмена"
                                    Layout.fillWidth: true
                                    onClicked: root.cancelPending()
                                }
                            }
                        }
                    }
                }
            }

            // ===== thrusters
            DpPanel {
                title: "Thruster main"
                rightTitle: "ALLOC: VARIABLE"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 330
                Layout.minimumWidth: 260
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 2
                    Canvas {
                        id: mimic
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 200
                        onPaint: {
                            var ctx = getContext("2d");
                            ctx.reset();
                            var w = width, h = height;
                            var sx = (w - 70) / 20.0, sy = (h - 70) / 88.0;
                            var cx = w / 2 - 30, cy = h / 2;
                            function scr(lx, ly) {
                                return [cx + ly * sx, cy - lx * sy];
                            }
                            // корпус с градиентом
                            var pts = [[44, 0], [20, 9], [-34, 9], [-44, 4],
                                       [-44, -4], [-34, -9], [20, -9]];
                            var g = ctx.createLinearGradient(cx, cy - 44 * sy,
                                                             cx, cy + 44 * sy);
                            g.addColorStop(0, root.pal.disk1);
                            g.addColorStop(1, root.pal.disk2);
                            ctx.beginPath();
                            for (var j = 0; j < pts.length; j++) {
                                var pp = scr(pts[j][0], pts[j][1]);
                                if (j === 0) ctx.moveTo(pp[0], pp[1]);
                                else ctx.lineTo(pp[0], pp[1]);
                            }
                            ctx.closePath();
                            ctx.fillStyle = g;
                            ctx.fill();
                            ctx.strokeStyle = root.pal.dim;
                            ctx.lineWidth = 1.5;
                            ctx.stroke();
                            if (!root.hasData()) return;
                            var s = Telem.state;
                            ctx.font = "10px Menlo";
                            ctx.textAlign = "center";
                            for (var i = 0; i < s.thrusters.length; i++) {
                                var t = s.thrusters[i];
                                var pos = scr(t.lx != null ? t.lx : 0,
                                              t.ly != null ? t.ly : 0);
                                var au = Math.abs(t.u_pct);
                                var col = t.failed ? root.red
                                        : au < 60 ? root.green
                                        : au < 90 ? root.yellow : root.orange;
                                ctx.beginPath();
                                ctx.arc(pos[0], pos[1], 13, 0, 2 * Math.PI);
                                ctx.fillStyle = root.pal.panel2;
                                ctx.fill();
                                ctx.strokeStyle = col;
                                ctx.lineWidth = 1.5;
                                ctx.stroke();
                                // стрелка азимута
                                ctx.save();
                                ctx.translate(pos[0], pos[1]);
                                ctx.rotate(t.alpha_deg * Math.PI / 180);
                                ctx.strokeStyle = col;
                                ctx.lineWidth = 2.5;
                                ctx.beginPath();
                                ctx.moveTo(0, 0);
                                ctx.lineTo(0, -12);
                                ctx.stroke();
                                ctx.restore();
                                // столбик тяги
                                var bh = Math.min(au / 100, 1) * 34;
                                ctx.fillStyle = col;
                                if (t.u_pct >= 0)
                                    ctx.fillRect(pos[0] + 17,
                                                 pos[1] + 15 - bh, 7, bh);
                                else
                                    ctx.fillRect(pos[0] + 17, pos[1] + 15,
                                                 7, bh);
                                ctx.fillStyle = root.pal.dim;
                                ctx.fillText(t.name != null ? t.name
                                                            : "ДРУ" + (i + 1),
                                             pos[0], pos[1] - 20);
                                ctx.fillStyle = root.pal.txt;
                                ctx.fillText(t.failed ? "ОТКАЗ"
                                    : t.alpha_deg.toFixed(0) + "° "
                                      + au.toFixed(0) + "%",
                                    pos[0], pos[1] + 30);
                            }
                        }
                    }
                    KV { k: "Resultant force"; v: root.hasData() ? Telem.state.resultant.force_kn.toFixed(0) + " kN" : "—" }
                    KV { k: "Resultant direction"; v: root.hasData() ? Telem.state.resultant.dir_deg.toFixed(1) + "°" : "—" }
                    KV { k: "Resultant moment"; v: root.hasData() ? Telem.state.resultant.moment_knm.toFixed(0) + " kNm" : "—" }
                }
            }

            // ===== wind + датчики (правая колонка)
            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 300
                Layout.minimumWidth: 240
                spacing: 6

            DpPanel {
                title: "Wind"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 2
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 2
                    Canvas {
                        id: windRose
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 130
                        onPaint: {
                            var ctx = getContext("2d");
                            ctx.reset();
                            var w = width, h = height;
                            var R = Math.max(20, Math.min(w, h) / 2 - 8);
                            var cx = w / 2, cy = h / 2;
                            var g = ctx.createLinearGradient(cx, cy - R, cx, cy + R);
                            g.addColorStop(0, root.pal.disk1);
                            g.addColorStop(1, root.pal.disk2);
                            ctx.fillStyle = g;
                            ctx.strokeStyle = root.pal.line;
                            ctx.beginPath();
                            ctx.arc(cx, cy, R, 0, 2 * Math.PI);
                            ctx.fill();
                            ctx.stroke();
                            ctx.font = Math.max(8, R / 11) + "px Menlo";
                            ctx.textAlign = "center";
                            for (var a = 0; a < 360; a += 30) {
                                var sa = Math.sin(a * Math.PI / 180);
                                var ca = Math.cos(a * Math.PI / 180);
                                ctx.strokeStyle = root.pal.dim;
                                ctx.beginPath();
                                ctx.moveTo(cx + 0.9 * R * sa, cy - 0.9 * R * ca);
                                ctx.lineTo(cx + R * sa, cy - R * ca);
                                ctx.stroke();
                                ctx.fillStyle = a === 0 ? root.red : root.pal.dim;
                                ctx.fillText(String(a), cx + 0.72 * R * sa,
                                             cy - 0.72 * R * ca + 4);
                            }
                            if (!root.hasData()) return;
                            var s = Telem.state;
                            // --- диаграмма возможностей (K-Pos №10):
                            // поляры предельного ветра от носа, повёрнутые
                            // на текущий курс; шкала 60 kn = полный радиус
                            var cap = s.capability;
                            if (cap && cap.betas && cap.betas.length > 0) {
                                var FULL_KN = 60.0;
                                function envRadius(v) {
                                    return Math.min(v / FULL_KN, 1.0) * R;
                                }
                                function drawEnv(vals, color, width) {
                                    ctx.beginPath();
                                    for (var k = 0; k <= cap.betas.length; k++) {
                                        var i = k % cap.betas.length;
                                        var brg = (s.hdg_deg + cap.betas[i])
                                                  * Math.PI / 180;
                                        var rr = envRadius(vals[i]);
                                        var px = cx + rr * Math.sin(brg);
                                        var py = cy - rr * Math.cos(brg);
                                        if (k === 0) ctx.moveTo(px, py);
                                        else ctx.lineTo(px, py);
                                    }
                                    ctx.closePath();
                                    ctx.strokeStyle = color;
                                    ctx.lineWidth = width;
                                    ctx.stroke();
                                }
                                // зона, выдерживаемая после ещё одного
                                // отказа — мягкая заливка
                                ctx.save();
                                ctx.globalAlpha = 0.10;
                                ctx.beginPath();
                                for (var k2 = 0; k2 <= cap.betas.length; k2++) {
                                    var i2 = k2 % cap.betas.length;
                                    var brg2 = (s.hdg_deg + cap.betas[i2])
                                               * Math.PI / 180;
                                    var rr2 = envRadius(cap.worst[i2]);
                                    var px2 = cx + rr2 * Math.sin(brg2);
                                    var py2 = cy - rr2 * Math.cos(brg2);
                                    if (k2 === 0) ctx.moveTo(px2, py2);
                                    else ctx.lineTo(px2, py2);
                                }
                                ctx.closePath();
                                ctx.fillStyle = root.green;
                                ctx.fill();
                                ctx.restore();
                                drawEnv(cap.intact, root.green, 1.2);
                                drawEnv(cap.worst, root.orange, 1.8);
                                // маркер текущего ветра (по измеренному):
                                // точка на подветренной стороне; красная,
                                // если за пределом «ещё один отказ»
                                var wr = Math.min(s.wind_meas.speed_kn
                                                  / FULL_KN, 1.0) * R;
                                var wbrg = s.wind_dir * Math.PI / 180;
                                var okM = cap.margin_kn === null
                                          || cap.margin_kn >= 0;
                                ctx.beginPath();
                                ctx.arc(cx + wr * Math.sin(wbrg),
                                        cy - wr * Math.cos(wbrg),
                                        4, 0, 2 * Math.PI);
                                ctx.fillStyle = okM ? root.cyan : root.red;
                                ctx.fill();
                            }
                            ctx.save();
                            ctx.translate(cx, cy);
                            ctx.rotate(s.wind_dir * Math.PI / 180);
                            ctx.shadowColor = root.cyan;
                            ctx.shadowBlur = 10;
                            ctx.strokeStyle = root.cyan;
                            ctx.lineWidth = 2;
                            ctx.beginPath();
                            ctx.moveTo(0, 0.75 * R);
                            ctx.lineTo(0, 0.12 * R);
                            ctx.stroke();
                            ctx.beginPath();
                            ctx.moveTo(0, 0);
                            ctx.lineTo(0.09 * R, 0.18 * R);
                            ctx.lineTo(-0.09 * R, 0.18 * R);
                            ctx.closePath();
                            ctx.fillStyle = root.cyan;
                            ctx.fill();
                            ctx.restore();
                        }
                    }
                    KV { k: "Измеренный"; v: root.hasData() ? Telem.state.wind_meas.speed_kn.toFixed(1) + " kn  " + Telem.state.wind_meas.dir_deg.toFixed(0) + "°" : "—" }
                    KV { k: "Истинный"; v: root.hasData() ? Telem.state.wind_true.speed_kn.toFixed(1) + " kn  " + Telem.state.wind_true.dir_deg.toFixed(0) + "°" : "—" }
                    KV { k: "Волна Hs / Tp"; v: root.hasData() ? Telem.state.hs.toFixed(1) + " м / " + Telem.state.tp.toFixed(0) + " с" : "—" }
                    KV { k: "От носа / пр. борта"; v: root.hasData() ? Telem.state.wind_rel.bow_kn.toFixed(1) + " / " + Telem.state.wind_rel.stbd_kn.toFixed(1) + " kn" : "—" }
                    KV { k: "Предел DP-2 (ещё отказ)"; v: root.hasData() && Telem.state.capability && Telem.state.capability.limit_kn != null ? Telem.state.capability.limit_kn.toFixed(0) + " kn" : "—" }
                    KV {
                        k: "Запас по ветру"
                        v: root.hasData() && Telem.state.capability && Telem.state.capability.margin_kn != null ? (Telem.state.capability.margin_kn >= 0 ? "+" : "") + Telem.state.capability.margin_kn.toFixed(0) + " kn" : "—"
                        vColor: !root.hasData() || !Telem.state.capability || Telem.state.capability.margin_kn == null ? root.pal.txt
                              : Telem.state.capability.margin_kn < 0 ? root.red
                              : Telem.state.capability.margin_kn < 10 ? root.yellow
                              : root.green
                    }
                }
            }

            // ===== sensors
            DpPanel {
                title: "Опорные системы"
                rightTitle: "вес · σ / откл."
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 3
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 2
                    Repeater {
                        model: root.hasData() && Telem.state.sensors.prs_detail
                               ? Telem.state.sensors.prs_detail : []
                        delegate: SysRow {
                            Layout.fillWidth: true
                            kind: "prs"; idx: index
                            name: modelData.name
                            st: modelData.state
                            reason: modelData.reason
                            disagree: modelData.disagree
                            info: modelData.weight_pct.toFixed(0) + "%  σ"
                                  + modelData.sigma_m.toFixed(1)
                                  + (modelData.age_s != null
                                     ? "  " + modelData.age_s.toFixed(1)
                                       + "с" : "")
                        }
                    }
                    Repeater {
                        model: root.hasData() && Telem.state.sensors.gyro_detail
                               ? Telem.state.sensors.gyro_detail : []
                        delegate: SysRow {
                            Layout.fillWidth: true
                            kind: "gyro"; idx: index
                            name: modelData.name
                            st: modelData.state
                            reason: modelData.reason
                            disagree: modelData.disagree
                            used: modelData.used
                            info: (modelData.dev_deg >= 0 ? "+" : "")
                                  + modelData.dev_deg.toFixed(1) + "°"
                                  + (modelData.age_s != null
                                     ? "  " + modelData.age_s.toFixed(1)
                                       + "с" : "")
                        }
                    }
                    KV { k: "Отклонение позиции"
                        v: root.hasData() ? Telem.state.pos_err.toFixed(2) + " м" : "—"
                        vColor: !root.hasData() ? root.pal.txt
                              : Telem.state.pos_err > Telem.state.alarm_pos ? root.red
                              : Telem.state.pos_err > Telem.state.warn_pos ? root.yellow
                              : root.green
                    }
                    KV { k: "N / E"; v: root.hasData() ? Telem.state.err_n.toFixed(2) + " / " + Telem.state.err_e.toFixed(2) + " м" : "—" }
                    KV { k: "Bias (LF смещение)"; v: root.hasData() ? Telem.state.bias_kn.toFixed(1) + " kN" : "—" }
                }
            }

            // ===== электростанция (PMS)
            DpPanel {
                title: "Электростанция (PMS)"
                rightTitle: root.hasData() && Telem.state.plant
                    ? (Telem.state.plant.tie_closed ? "СА замкнут"
                                                    : "СА РАЗОМКНУТ")
                    : ""
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredHeight: 1.6
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 3
                    Repeater {
                        model: root.hasData() && Telem.state.plant
                               ? Telem.state.plant.buses : []
                        delegate: ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 1
                            property var bus: modelData
                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 6
                                Text {
                                    text: "Шина " + bus.name
                                    color: bus.dead ? root.red : root.pal.txt
                                    font.pixelSize: 11; font.bold: true
                                }
                                Rectangle {
                                    Layout.fillWidth: true
                                    implicitHeight: 10
                                    color: root.pal.panel2
                                    border.color: root.pal.line
                                    Rectangle {
                                        anchors.left: parent.left
                                        anchors.top: parent.top
                                        anchors.bottom: parent.bottom
                                        width: bus.avail_kw > 0
                                            ? parent.width * Math.min(
                                                bus.load_kw / bus.avail_kw, 1)
                                            : (bus.load_kw > 0
                                               ? parent.width : 0)
                                        color: bus.dead ? root.red
                                            : bus.load_kw > 0.85 * bus.avail_kw
                                              ? root.orange : root.green
                                    }
                                }
                                Text {
                                    text: bus.load_kw.toFixed(0) + " / "
                                          + bus.avail_kw.toFixed(0) + " кВт"
                                    color: root.pal.dim; font.pixelSize: 10
                                }
                            }
                            RowLayout {
                                spacing: 4
                                Repeater {
                                    model: root.hasData()
                                           && Telem.state.plant
                                           ? Telem.state.plant.gens : []
                                    delegate: Rectangle {
                                        required property var modelData
                                        visible: modelData.bus === bus.name
                                        implicitWidth: 40; implicitHeight: 15
                                        radius: 2
                                        color: modelData.state === "online"
                                               ? root.green
                                          : modelData.state === "starting"
                                            ? root.yellow
                                          : modelData.state === "failed"
                                            ? root.red : root.pal.panel2
                                        border.color: root.pal.line
                                        Text {
                                            anchors.centerIn: parent
                                            text: modelData.state === "starting"
                                                ? modelData.name + "…"
                                                : modelData.name
                                            color: modelData.state === "stopped"
                                                   ? root.pal.dim : "#101418"
                                            font.pixelSize: 9; font.bold: true
                                        }
                                    }
                                }
                            }
                        }
                    }
                    KV { k: "Нагрузка / доступно"
                         v: root.hasData() && Telem.state.plant
                            ? Telem.state.plant.load_kw.toFixed(0) + " / "
                              + Telem.state.plant.avail_kw.toFixed(0) + " кВт"
                            : "—" }
                    KV {
                        k: "Резерв мощности"
                        v: root.hasData() && Telem.state.plant
                           ? Telem.state.plant.reserve_pct.toFixed(0) + " %"
                           : "—"
                        vColor: !root.hasData() || !Telem.state.plant
                                ? root.pal.txt
                              : Telem.state.plant.reserve_pct < 15 ? root.red
                              : Telem.state.plant.reserve_pct < 25
                                ? root.yellow : root.green
                    }
                    KV {
                        k: "Ограничение тяги"
                        v: root.hasData() && Telem.state.plant
                           && Telem.state.plant.limit_mask.some(
                               function(x) { return x; }) ? "АКТИВНО" : "нет"
                        vColor: root.hasData() && Telem.state.plant
                                && Telem.state.plant.limit_mask.some(
                                    function(x) { return x; })
                                ? root.orange : root.green
                    }
                }
            }
            }
        }

        // ---------------- нижний ряд
        RowLayout {
            Layout.fillWidth: true
            Layout.preferredHeight: 210
            Layout.maximumHeight: 240
            spacing: 6

            DpPanel {
                title: "Тренды: ошибка позиции · ветер · суммарная тяга"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 460
                Layout.minimumWidth: 320
                Canvas {
                    id: trendsCanvas
                    anchors.fill: parent
                    onPaint: {
                        var ctx = getContext("2d");
                        ctx.reset();
                        var w = width, h = height;
                        ctx.fillStyle = root.pal.panel2;
                        ctx.fillRect(0, 0, w, h);
                        ctx.strokeStyle = root.pal.line;
                        for (var i = 1; i < 4; i++) {
                            ctx.beginPath();
                            ctx.moveTo(0, h * i / 4);
                            ctx.lineTo(w, h * i / 4);
                            ctx.stroke();
                        }
                        var d = root.trend, n = d.length;
                        if (n < 2) return;
                        var series = [[0, 8.0, root.orange],
                                      [1, 45.0, root.cyan],
                                      [2, 400.0, root.green]];
                        for (var si = 0; si < 3; si++) {
                            var idx = series[si][0], mx = series[si][1];
                            var col = series[si][2];
                            ctx.beginPath();
                            for (var j = 0; j < n; j++) {
                                var x = j / (n - 1) * w;
                                var y = h - Math.min(d[j][idx] / mx, 1)
                                        * h * 0.94 - 3;
                                if (j === 0) ctx.moveTo(x, y);
                                else ctx.lineTo(x, y);
                            }
                            ctx.strokeStyle = col;
                            ctx.lineWidth = 1.6;
                            ctx.stroke();
                            ctx.lineTo(w, h);
                            ctx.lineTo(0, h);
                            ctx.closePath();
                            ctx.fillStyle = Qt.rgba(col.r, col.g, col.b, 0.12);
                            ctx.fill();
                        }
                    }
                }
            }

            DpPanel {
                title: "Аварийный журнал"
                rightTitle: "клик = квитировать"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 320
                Layout.minimumWidth: 240
                ListView {
                    id: alarmView
                    anchors.fill: parent
                    clip: true
                    spacing: 2
                    model: root.hasData() ? Telem.state.alarms : []
                    delegate: Rectangle {
                        // клик по аварии = её квитирование (MG 6.1.10);
                        // пояснение раскрывается при наведении
                        readonly property bool hov: rowMa.containsMouse
                                                     && !!modelData.hint
                        width: alarmView.width
                        height: hov
                                ? mainTxt.implicitHeight
                                  + hintTxt.implicitHeight + 10
                                : 22
                        radius: 3
                        color: root.pal.panel2
                        opacity: modelData.acked ? 0.45 : 1.0
                        Behavior on height { NumberAnimation { duration: 120 } }
                        Rectangle {
                            width: 3
                            height: parent.height
                            color: modelData.level === 2 ? root.red
                                 : modelData.level === 1 ? root.yellow
                                 : root.pal.line
                        }
                        Text {
                            id: mainTxt
                            x: 10
                            width: parent.width - 16
                            anchors.top: parent.top
                            anchors.topMargin: 4
                            text: (modelData.clock ? modelData.clock
                                  : modelData.t.toFixed(0) + " с")
                                  + "   " + modelData.text
                                  + (modelData.hint
                                     ? (parent.hov ? "  ▾" : "  ▸") : "")
                                  + (modelData.acked ? "   ✓" : "")
                            wrapMode: Text.Wrap
                            font.pixelSize: 11
                            font.family: "Menlo"
                            color: modelData.acked ? root.pal.dim
                                 : modelData.level === 2 ? root.red
                                 : modelData.level === 1 ? root.yellow
                                 : root.pal.txt
                        }
                        Text {
                            id: hintTxt
                            x: 10
                            width: parent.width - 16
                            anchors.top: mainTxt.bottom
                            anchors.topMargin: 3
                            visible: parent.hov
                            text: modelData.hint ? modelData.hint : ""
                            wrapMode: Text.Wrap
                            font.pixelSize: 10
                            color: root.pal.dim
                        }
                        MouseArea {
                            id: rowMa
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: modelData.acked
                                         ? Qt.ArrowCursor
                                         : Qt.PointingHandCursor
                            onClicked: {
                                if (!modelData.acked)
                                    Telem.cmd({ "action": "ack_one",
                                                "id": modelData.id });
                            }
                        }
                    }
                }
            }

            DpPanel {
                title: root.instrLock ? "Пульт инструктора 🔒"
                                      : "Пульт инструктора"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 330
                Layout.minimumWidth: 260
                Item {
                    anchors.fill: parent
                ColumnLayout {
                    anchors.fill: parent
                    visible: !root.instrLock
                    spacing: 4
                    RowLayout {
                        spacing: 4
                        // кнопки-переключатели отказ ↔ возврат: по одной
                        // на движитель активного профиля (4 или 8)
                        Repeater {
                            model: root.hasData() ? Telem.state.thrusters.length : 0
                            DpButton {
                                required property int index
                                property bool f: root.hasData()
                                    && Telem.state.thrusters.length > index
                                    && Telem.state.thrusters[index].failed
                                property string nm: root.hasData()
                                    && Telem.state.thrusters.length > index
                                    && Telem.state.thrusters[index].name != null
                                    ? Telem.state.thrusters[index].name
                                    : "ДРУ" + (index + 1)
                                text: f ? nm + "↩" : nm; accent: true
                                onClicked: Telem.cmd(f ? { "action": "recover", "kind": "thruster", "idx": index } : { "action": "fail", "kind": "thruster", "idx": index })
                            }
                        }
                        DpButton {
                            property bool f: root.hasData() && Telem.state.sensors.prs[2]
                            text: f ? "PRS3↩" : "PRS3"; accent: true
                            onClicked: Telem.cmd(f ? { "action": "recover", "kind": "prs", "idx": 2 } : { "action": "fail", "kind": "prs", "idx": 2 })
                        }
                        DpButton {
                            property bool f: root.hasData() && Telem.state.sensors.gyro[0]
                            text: f ? "ГИРО1↩" : "ГИРО1"; accent: true
                            onClicked: Telem.cmd(f ? { "action": "recover", "kind": "gyro", "idx": 0 } : { "action": "fail", "kind": "gyro", "idx": 0 })
                        }
                    }
                    // электростанция: отказы ДГ, шин, секционный автомат
                    RowLayout {
                        spacing: 4
                        Repeater {
                            model: root.hasData() && Telem.state.plant
                                   ? Telem.state.plant.gens.length : 0
                            DpButton {
                                required property int index
                                property bool f: root.hasData()
                                    && Telem.state.plant
                                    && Telem.state.plant.gens[index].state === "failed"
                                text: (f ? "ДГ" + (index + 1) + "↩"
                                         : "ДГ" + (index + 1))
                                accent: true
                                onClicked: Telem.cmd(f ? { "action": "recover", "kind": "gen", "idx": index } : { "action": "fail", "kind": "gen", "idx": index })
                            }
                        }
                        DpButton {
                            property bool f: root.hasData() && Telem.state.plant
                                && Telem.state.plant.buses[0].dead
                            text: f ? "ШинаА↩" : "ШинаА"; accent: true
                            onClicked: Telem.cmd(f ? { "action": "recover", "kind": "bus", "idx": 0 } : { "action": "fail", "kind": "bus", "idx": 0 })
                        }
                        DpButton {
                            property bool f: root.hasData() && Telem.state.plant
                                && Telem.state.plant.buses[1].dead
                            text: f ? "ШинаБ↩" : "ШинаБ"; accent: true
                            onClicked: Telem.cmd(f ? { "action": "recover", "kind": "bus", "idx": 1 } : { "action": "fail", "kind": "bus", "idx": 1 })
                        }
                        DpButton {
                            property bool f: root.hasData() && Telem.state.plant
                                && Telem.state.plant.tie_closed
                            text: f ? "СА ⚡" : "СА ○"
                            onClicked: Telem.cmd({ "action": "tie", "closed": !f })
                        }
                    }
                    RowLayout {
                        spacing: 4
                        Text { text: "Сбой:"; color: root.pal.dim; font.pixelSize: 11 }
                        DpButton {
                            text: "PRS1 завис"
                            onClicked: Telem.cmd({ "action": "fail", "kind": "prs", "idx": 0, "ftype": "frozen" })
                        }
                        DpButton {
                            text: "PRS2 шум"
                            onClicked: Telem.cmd({ "action": "fail", "kind": "prs", "idx": 1, "ftype": "noisy" })
                        }
                        DpButton {
                            text: "PRS3 плывёт"
                            onClicked: Telem.cmd({ "action": "fail", "kind": "prs", "idx": 2, "ftype": "drift" })
                        }
                        DpButton {
                            text: "ГИРО2 шум"
                            onClicked: Telem.cmd({ "action": "fail", "kind": "gyro", "idx": 1, "ftype": "noisy" })
                        }
                        DpButton {
                            text: "Датчики ↩"
                            onClicked: {
                                for (var i = 0; i < 3; i++)
                                    Telem.cmd({ "action": "recover", "kind": "prs", "idx": i })
                                for (var j = 0; j < 2; j++)
                                    Telem.cmd({ "action": "recover", "kind": "gyro", "idx": j })
                            }
                        }
                    }
                    Text { text: "Ветер: " + windV.value.toFixed(1) + " м/с"; color: root.pal.dim; font.pixelSize: 11 }
                    Slider {
                        id: windV
                        Layout.fillWidth: true
                        from: 0; to: 20; stepSize: 0.5; value: 10
                        onMoved: Telem.cmd({ "action": "wind",
                                             "speed_ms": value,
                                             "dir_deg": windD.value })
                    }
                    Text { text: "Направление ветра: " + windD.value.toFixed(0) + "°"; color: root.pal.dim; font.pixelSize: 11 }
                    Slider {
                        id: windD
                        Layout.fillWidth: true
                        from: 0; to: 360; stepSize: 5; value: 30
                        onMoved: Telem.cmd({ "action": "wind",
                                             "speed_ms": windV.value,
                                             "dir_deg": value })
                    }
                    Text { text: "Hs: " + hsS.value.toFixed(1) + " м"; color: root.pal.dim; font.pixelSize: 11 }
                    Slider {
                        id: hsS
                        Layout.fillWidth: true
                        from: 0; to: 6; stepSize: 0.5; value: 2
                        onMoved: Telem.cmd({ "action": "hs", "hs": value })
                    }
                    Text { text: "Ускорение: " + spdS.value.toFixed(0) + "×"; color: root.pal.dim; font.pixelSize: 11 }
                    Slider {
                        id: spdS
                        Layout.fillWidth: true
                        from: 1; to: 20; stepSize: 1; value: 5
                        onMoved: Telem.cmd({ "action": "speed", "value": value })
                    }
                    RowLayout {
                        spacing: 4
                        DpButton { text: "Пауза"; onClicked: Telem.cmd({ "action": "pause" }) }
                        DpButton { text: "Курс −10°"; onClicked: Telem.cmd({ "action": "heading_bump", "d_deg": -10 }) }
                        DpButton { text: "Курс +10°"; onClicked: Telem.cmd({ "action": "heading_bump", "d_deg": 10 }) }
                        DpConfirmButton { label: "Сброс"; confirmLabel: "СБРОС?"; danger: true; onConfirmed: Telem.cmd({ "action": "reset" }) }
                    }
                    RowLayout {
                        spacing: 4
                        Text { text: "Watch circle, м:"; color: root.pal.dim; font.pixelSize: 11 }
                        SpinBox {
                            id: wcW
                            from: 1; to: 50; value: 3
                            palette.text: root.pal.txt
                            palette.base: root.pal.panel2
                        }
                        SpinBox {
                            id: wcA
                            from: 2; to: 80; value: 5
                            palette.text: root.pal.txt
                            palette.base: root.pal.panel2
                        }
                        DpButton {
                            text: "Задать"
                            onClicked: Telem.cmd({ "action": "limits",
                                                   "warn": wcW.value,
                                                   "alarm": wcA.value })
                        }
                    }
                    DpButton {
                        text: "🔒 Заблокировать имитатор"
                        Layout.fillWidth: true
                        onClicked: root.instrLock = true
                    }
                }
                // ---- замок имитатора: удержание 2 с для доступа
                Rectangle {
                    anchors.fill: parent
                    visible: root.instrLock
                    color: root.pal.panel2
                    radius: 4
                    Column {
                        anchors.centerIn: parent
                        spacing: 10
                        Text {
                            anchors.horizontalCenter: parent.horizontalCenter
                            text: "🔒 ИМИТАТОР ЗАБЛОКИРОВАН"
                            color: root.pal.dim
                            font.pixelSize: 13
                            font.bold: true
                            font.letterSpacing: 1
                        }
                        Text {
                            anchors.horizontalCenter: parent.horizontalCenter
                            text: "Нажмите и удерживайте 2 с для доступа"
                            color: root.pal.dim
                            font.pixelSize: 11
                        }
                        Rectangle {
                            anchors.horizontalCenter: parent.horizontalCenter
                            width: 180; height: 6; radius: 3
                            color: root.pal.line
                            Rectangle {
                                width: parent.width * root.lockProg
                                height: parent.height; radius: 3
                                color: root.orange
                            }
                        }
                    }
                    MouseArea {
                        anchors.fill: parent
                        onPressed: lockAnim.restart()
                        onReleased: { lockAnim.stop(); root.lockProg = 0 }
                    }
                    NumberAnimation {
                        id: lockAnim
                        target: root
                        property: "lockProg"
                        from: 0; to: 1
                        duration: 2000
                        onFinished: {
                            root.instrLock = false;
                            root.lockProg = 0;
                        }
                    }
                }
                }
            }

            // ===== джойстик (MAN JOY / AUTO JOY)
            DpPanel {
                title: "Джойстик"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.preferredWidth: 250
                Layout.minimumWidth: 210
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 4
                    RowLayout {
                        spacing: 4
                        DpConfirmButton {
                            label: "АВТО"
                            active: root.hasData() && Telem.state.mode === "auto"
                            onConfirmed: Telem.cmd({ "action": "mode", "mode": "auto" })
                        }
                        DpConfirmButton {
                            label: "MAN"
                            active: root.hasData() && Telem.state.mode === "man_joy"
                            onConfirmed: Telem.cmd({ "action": "mode", "mode": "man_joy" })
                        }
                        DpConfirmButton {
                            label: "AUTO JOY"
                            active: root.hasData() && Telem.state.mode === "auto_joy"
                            onConfirmed: Telem.cmd({ "action": "mode", "mode": "auto_joy" })
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 6
                        // пультовая площадка: вверх = вперёд, вправо = на правый борт
                        Canvas {
                            id: joyPad
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Layout.minimumHeight: 110
                            property real jx: 0    // боковая (вправо +)
                            property real jy: 0    // продольная (вперёд +)
                            function sendJoy() {
                                Telem.cmd({ "action": "joystick",
                                            "fx": jy, "fy": jx,
                                            "m": yawJoy.value })
                            }
                            onPaint: {
                                var ctx = getContext("2d");
                                ctx.reset();
                                var w = width, h = height;
                                var R = Math.max(20, Math.min(w, h) / 2 - 8);
                                var cx = w / 2, cy = h / 2;
                                ctx.fillStyle = root.pal.panel2;
                                ctx.strokeStyle = root.pal.line;
                                ctx.lineWidth = 1;
                                ctx.beginPath();
                                ctx.arc(cx, cy, R, 0, 2 * Math.PI);
                                ctx.fill(); ctx.stroke();
                                ctx.setLineDash([3, 3]);
                                ctx.beginPath();
                                ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
                                ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
                                ctx.stroke();
                                ctx.setLineDash([]);
                                ctx.fillStyle = root.pal.dim;
                                ctx.font = "9px Menlo";
                                ctx.textAlign = "center";
                                ctx.fillText("ВПЕРЁД", cx, cy - R + 10);
                                ctx.fillText("ПБ", cx + R - 12, cy + 3);
                                // ручка
                                var hx = cx + jx * (R - 12);
                                var hy = cy - jy * (R - 12);
                                ctx.beginPath();
                                ctx.arc(hx, hy, 10, 0, 2 * Math.PI);
                                ctx.fillStyle = root.cyan;
                                ctx.fill();
                                ctx.strokeStyle = root.pal.txt;
                                ctx.stroke();
                            }
                            MouseArea {
                                id: joyPadArea
                                anchors.fill: parent
                                function upd(mx, my) {
                                    var R = Math.max(20,
                                            Math.min(width, height) / 2 - 8) - 12;
                                    var dx = (mx - width / 2) / R;
                                    var dy = -(my - height / 2) / R;
                                    var m = Math.hypot(dx, dy);
                                    if (m > 1) { dx /= m; dy /= m; }
                                    joyPad.jx = dx; joyPad.jy = dy;
                                    joyPad.sendJoy();
                                    joyPad.requestPaint();
                                }
                                onPressed: function(mouse) {
                                    upd(mouse.x, mouse.y)
                                }
                                onPositionChanged: function(mouse) {
                                    if (pressed) upd(mouse.x, mouse.y)
                                }
                            }
                        }
                        // момент (рысканье): вертикальный ползунок
                        ColumnLayout {
                            spacing: 2
                            Text { text: "М"; color: root.pal.dim; font.pixelSize: 10; Layout.alignment: Qt.AlignHCenter }
                            Slider {
                                id: yawJoy
                                orientation: Qt.Vertical
                                Layout.fillHeight: true
                                from: -1; to: 1; value: 0; stepSize: 0.05
                                onMoved: joyPad.sendJoy()
                            }
                        }
                    }
                    RowLayout {
                        spacing: 4
                        DpButton {
                            text: "Центр"
                            onClicked: {
                                joyPad.jx = 0; joyPad.jy = 0;
                                yawJoy.value = 0;
                                joyPad.sendJoy();
                                joyPad.requestPaint();
                            }
                        }
                        Text {
                            color: root.pal.dim; font.pixelSize: 10
                            text: root.hasData() && Telem.state.joy
                                  ? "F " + (Telem.state.joy[0] * 100).toFixed(0)
                                    + " / " + (Telem.state.joy[1] * 100).toFixed(0)
                                    + "  M " + (Telem.state.joy[2] * 100).toFixed(0) + " %"
                                  : "—"
                        }
                    }
                }
            }
        }
    }
}

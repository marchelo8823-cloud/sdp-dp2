// Пульт оператора СДП (1920×1080). Слева — план положения, справа — курс и положение, режимы, задание точки,
// ветер и возмущение; внизу — движители, тревоги, датчики и журнал с командной строкой.
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import "."

ApplicationWindow {
    id: win
    width: 1920; height: 1080
    visible: true
    title: "СДП — пульт оператора" + (core.s.shipName ? " — " + core.s.shipName : "")
    color: Theme.bg
    font.family: Theme.font
    font.pixelSize: Theme.fs
    palette.window: Theme.bg; palette.windowText: Theme.text; palette.base: Theme.panel2; palette.text: Theme.text
    palette.button: Theme.panel2; palette.buttonText: Theme.text; palette.highlight: Theme.accent; palette.highlightedText: Theme.bg
    palette.mid: Theme.line; palette.dark: Theme.line; palette.light: Theme.panel2; palette.placeholderText: Theme.dim

    readonly property var s: core.s
    property real step: 1.0          // шаг сдвига точки, м
    property real hstep: 1.0         // шаг курса, °

    FileDialog { id: shipDlg; title: "Книга данных судна"; nameFilters: ["Книги Excel (*.xlsx)"]; onAccepted: core.loadShip(selectedFile) }
    FileDialog { id: simDlg; title: "Книга имитатора"; nameFilters: ["Книги Excel (*.xlsx)"]; onAccepted: core.simStart(selectedFile) }

    component Panel: Rectangle { color: Theme.panel; radius: 3 }
    component Caption: Label { color: Theme.dim; font.pixelSize: Theme.fsSmall; font.capitalization: Font.AllUppercase; font.letterSpacing: 1 }
    component Big: Label { color: Theme.text; font.pixelSize: Theme.fsBig; font.family: Theme.mono }
    component Btn: Button { font.pixelSize: Theme.fs; implicitHeight: 40 }
    // Кнопка с подтверждением (РС ч. XV 8.5.7, MSC.1/Circ.1580 3.4.1.4: одно случайное нажатие не должно приводить к потере
    // положения; как двойное нажатие в K-Pos и Mega-Guard): первое нажатие — кнопка мигает «ещё раз», второе в течение 3 с — команда.
    property string armedCmd: ""
    Timer { id: armT; interval: 3000; onTriggered: win.armedCmd = "" }
    Timer { id: blinkT; interval: 400; repeat: true; running: win.armedCmd !== ""; property bool on: false; onTriggered: on = !on; onRunningChanged: on = true }
    component ConfirmBtn: Btn {
        property string cmd
        property string label
        readonly property bool armed: win.armedCmd === cmd
        text: armed ? "Ещё раз: " + label : label
        Rectangle { anchors.fill: parent; radius: 3; color: Theme.warn; opacity: parent.armed && blinkT.on ? 0.45 : 0; z: 1 }
        onClicked: {
            if (armed) { win.armedCmd = ""; armT.stop(); core.command(cmd) }
            else { win.armedCmd = cmd; armT.restart() }
        }
    }
    readonly property bool posLossOrReturn: { for (let i = 0; i < core.alarms.length; ++i) { const c = core.alarms[i].code; if (c === "POS_DROPOUT" || c === "POS_ALL_REJ" || c === "POS_RETURN") return true } return false }

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 8; spacing: 8

        // ------------------------------------------------ верхняя строка
        Panel {
            Layout.fillWidth: true; Layout.preferredHeight: 56
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 12; anchors.rightMargin: 12; spacing: 14
                Rectangle {
                    implicitWidth: stl.implicitWidth + 24; implicitHeight: 38; radius: 3
                    color: Theme.stateColor(s.stateCode)
                    Label { id: stl; anchors.centerIn: parent; text: s.state || ""; color: Theme.bg; font.bold: true; font.pixelSize: 19 }
                }
                Label { text: s.mode || ""; color: Theme.text; font.pixelSize: 19 }
                Rectangle {
                    visible: s.sim === true; implicitWidth: siml.implicitWidth + 16; implicitHeight: 30; radius: 3; color: "transparent"; border.color: Theme.warn
                    Label { id: siml; anchors.centerIn: parent; text: "ИМИТАЦИЯ ×" + Theme.num(s.simSpeed, 0); color: Theme.warn; font.pixelSize: Theme.fs }
                }
                Label { text: s.shipName || "судно не загружено"; color: Theme.dim; font.pixelSize: Theme.fs; Layout.fillWidth: true }
                Label { text: "t = " + Theme.num(s.clock, 0) + " с"; color: Theme.dim; font.family: Theme.mono }
                Btn { text: "Открыть судно…"; onClicked: shipDlg.open(); enabled: s.stateCode !== 3 }
                Btn { text: s.sim ? "Остановить имитатор" : "Запустить имитатор…"; enabled: s.hasShip === true && s.stateCode !== 3
                      onClicked: s.sim ? core.command("sim stop") : simDlg.open() }
            }
        }

        // ------------------------------------------------ середина
        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 8

            PositionView {
                id: plan
                Layout.fillHeight: true; Layout.fillWidth: false; Layout.preferredWidth: 820
                s: core.s; hull: core.hull; thrusters: core.thrusters; track: core.track
            }

            ColumnLayout {
                Layout.fillHeight: true; Layout.fillWidth: false; Layout.preferredWidth: 500; Layout.maximumWidth: 500; spacing: 8
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: 236
                    GridLayout {
                        anchors.fill: parent; anchors.margins: 14; columns: 3; rowSpacing: 6; columnSpacing: 18
                        Caption { text: "Курс" } Caption { text: "Задан" } Caption { text: "Ошибка" }
                        Big { text: s.hdgValid ? Theme.deg(s.hdg) : "НЕТ"; color: s.hdgValid ? (s.hdgDR ? Theme.warn : Theme.text) : Theme.alarm }
                        Big { text: Theme.deg(s.refHdg); color: Theme.accent }
                        Big { text: Theme.num(s.errHdg, 1) + "°" }
                        Caption { text: "Положение от точки" } Caption { text: "вдоль / поперёк" } Caption { text: "Поворот" }
                        Big { text: s.posValid ? Theme.num(s.errDist, 2) + " м" : "НЕТ"; color: !s.posValid ? Theme.alarm : s.posDR ? Theme.warn : Theme.text }
                        Label { text: Theme.num(s.errAlong, 1) + " / " + Theme.num(s.errCross, 1) + " м"; color: Theme.text; font.pixelSize: 20; font.family: Theme.mono }
                        Label { text: Theme.num(s.rot, 1) + " °/мин"; color: Theme.text; font.pixelSize: 20; font.family: Theme.mono }
                        Caption { text: "Источник положения"; Layout.columnSpan: 3 }
                        Label { text: s.posDR ? "СЧИСЛЕНИЕ — нет измерений" : (s.posSource || "—"); color: s.posDR ? Theme.warn : Theme.text; Layout.columnSpan: 3; font.pixelSize: Theme.fs }
                    }
                }
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: win.posLossOrReturn ? 244 : 196
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 12; spacing: 8
                        Caption { text: "Режим" }
                        RowLayout {
                            spacing: 8
                            ConfirmBtn { label: "Ожидание"; cmd: "standby"; highlighted: s.modeCode === 0; Layout.fillWidth: true }
                            ConfirmBtn { label: "Курс"; cmd: "heading"; highlighted: s.modeCode === 3; enabled: s.stateCode >= 2; Layout.fillWidth: true }
                            ConfirmBtn { label: "Позиция"; cmd: "position"; highlighted: s.modeCode === 4; enabled: s.stateCode >= 2; Layout.fillWidth: true }
                        }
                        RowLayout {
                            spacing: 8
                            Btn { text: "Взять все движители"; enabled: s.hasShip === true; onClicked: core.command("thr_take all"); Layout.fillWidth: true }
                            ConfirmBtn { label: "Снять все"; cmd: "thr_release all"; enabled: s.hasShip === true; Layout.fillWidth: true }
                        }
                        Btn { text: "Держать здесь (не возвращаться в точку)"; visible: s.modeCode === 4 && win.posLossOrReturn; Layout.fillWidth: true
                              palette.button: Theme.warn; palette.buttonText: "#1a1a1a"; font.bold: true; onClicked: core.command("here") }
                        Label { color: Theme.dim; font.pixelSize: Theme.fsSmall; wrapMode: Text.Wrap; Layout.fillWidth: true
                                text: win.armedCmd !== "" ? "Нажмите ту же кнопку ещё раз в течение 3 с, чтобы подтвердить." : s.stateCode === 0 ? "Загрузите судно." : s.stateCode === 1 ? "Ядро ждёт курс и положение от датчиков." : s.stateCode === 2 ? "Возьмите движители и включите режим." : "Режим включён: изменения точки и курса — ниже." }
                    }
                }
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: 168
                    enabled: s.stateCode === 3
                    opacity: enabled ? 1 : 0.5
                    GridLayout {
                        anchors.fill: parent; anchors.margins: 12; columns: 5; rowSpacing: 6; columnSpacing: 6
                        Caption { text: "Сдвиг точки"; Layout.columnSpan: 3 } Caption { text: "Курс"; Layout.columnSpan: 2 }
                        Item { width: 1 } Btn { text: "С ↑"; onClicked: core.command("move " + step + " 0") } Item { width: 1 }
                        Btn { text: "−" + hstep + "°"; onClicked: core.command("move 0 0 " + (-hstep)) } Btn { text: "+" + hstep + "°"; onClicked: core.command("move 0 0 " + hstep) }
                        Btn { text: "← З"; onClicked: core.command("move 0 " + (-step)) } Btn { text: "Ю ↓"; onClicked: core.command("move " + (-step) + " 0") } Btn { text: "В →"; onClicked: core.command("move 0 " + step) }
                        ComboBox { model: ["1°", "5°", "10°"]; onActivated: hstep = [1, 5, 10][currentIndex]; Layout.columnSpan: 2; Layout.fillWidth: true }
                        ComboBox { model: ["1 м", "5 м", "10 м"]; onActivated: step = [1, 5, 10][currentIndex]; Layout.columnSpan: 3; Layout.fillWidth: true }
                        Btn { text: "Точка = здесь"; Layout.columnSpan: 2; Layout.fillWidth: true
                              onClicked: core.command("set " + s.n.toFixed(2) + " " + s.e.toFixed(2) + " " + s.hdg.toFixed(1)) }
                    }
                }
            }

            // ------------------------------------------------ тревоги и датчики
            ColumnLayout {
                Layout.fillHeight: true; Layout.fillWidth: true; spacing: 8
                Panel {
                    Layout.fillWidth: true; Layout.fillHeight: true
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 10; spacing: 6
                        RowLayout {
                            Caption { text: "Тревоги (" + core.alarms.length + ")"; Layout.fillWidth: true }
                            Btn { text: "Квитировать"; implicitHeight: 32; onClicked: core.ack() }
                        }
                        ListView {
                            Layout.fillWidth: true; Layout.fillHeight: true; clip: true; spacing: 3
                            model: core.alarms
                            delegate: Rectangle {
                                width: ListView.view.width; height: al.implicitHeight + 12; radius: 2
                                color: modelData.level === 1 ? Qt.rgba(0.9, 0.33, 0.29, modelData.acked ? 0.18 : 0.38) : modelData.level === 2 ? Qt.rgba(0.89, 0.7, 0.24, modelData.acked ? 0.12 : 0.28) : Theme.panel2
                                border.color: Theme.levelColor(modelData.level); border.width: modelData.acked ? 1 : 2
                                Label { id: al; anchors.fill: parent; anchors.margins: 6; wrapMode: Text.Wrap; color: Theme.text; font.pixelSize: Theme.fsSmall
                                        text: "<b>" + modelData.code + "</b>  " + modelData.text; textFormat: Text.StyledText }
                            }
                            Label { visible: core.alarms.length === 0; text: "Тревог нет"; color: Theme.ok; anchors.centerIn: parent }
                        }
                    }
                }
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: 196
                    GridLayout {
                        anchors.fill: parent; anchors.margins: 12; columns: 2; rowSpacing: 5; columnSpacing: 14
                        Caption { text: "Ветер (откуда, от носа)" } Label { text: s.windValid ? Theme.num(s.windSpeed, 1) + " м/с (" + Theme.num(Theme.knots(s.windSpeed), 1) + " уз), " + Theme.num(s.windFromRel, 0) + "°" : "НЕТ"; color: s.windValid ? Theme.wind : Theme.warn; font.family: Theme.mono }
                        Caption { text: "Внешнее возмущение" } Label { text: Theme.num(Math.sqrt(s.distN * s.distN + s.distE * s.distE) / 1000, 0) + " кН, куда " + Theme.num((Math.atan2(s.distE, s.distN) * 180 / Math.PI + 360) % 360, 0) + "°"; color: Theme.dist; font.family: Theme.mono }
                        Caption { text: "Загрузка движителей" } Label { text: Theme.num(s.load * 100, 0) + " %"; color: s.load > 0.9 ? Theme.warn : Theme.text; font.family: Theme.mono }
                        Caption { text: "Мощность" } Label { text: Theme.num(s.powerKW, 0) + " кВт"; color: Theme.text; font.family: Theme.mono }
                        Caption { text: "Качка (крен / дифферент)" } Label { text: Theme.num(s.roll, 1) + "° / " + Theme.num(s.pitch, 1) + "°"; color: Theme.text; font.family: Theme.mono }
                        Caption { text: "Волнение (по МИО)" } Label { text: s.waveHs >= 0 ? Theme.num(s.waveHs, 1) + " м" : "не оценено"; color: Theme.text; font.family: Theme.mono }
                    }
                }
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: 190
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Caption { text: "Анализ последствий отказа" }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 6
                            Btn { text: "Класс 2"; implicitHeight: 28; Layout.fillWidth: true; highlighted: s.caClass === 2; enabled: s.hasShip === true; onClicked: core.command("class 2") }
                            Btn { text: "Класс 3"; implicitHeight: 28; Layout.fillWidth: true; highlighted: s.caClass === 3; enabled: s.hasShip === true; onClicked: core.command("class 3") }
                            Btn { text: "Выкл"; implicitHeight: 28; Layout.fillWidth: true; highlighted: s.caClass === 0; enabled: s.hasShip === true; onClicked: core.command("class off") }
                        }
                        Label { Layout.fillWidth: true; wrapMode: Text.Wrap; font.pixelSize: Theme.fsSmall
                                color: s.caClass === 0 ? Theme.dim : s.caAlarm ? Theme.alarm : s.caRan ? Theme.ok : Theme.dim
                                text: s.caClass === 0 ? "Выключен. Для DYNPOS-2/3 включите класс (РС ч. XV 8.9.5)."
                                    : !s.caRan ? "Класс " + s.caClass + ": расчёт через 60 с в режиме «Позиция» при неизменных точке и курсе."
                                    : (s.caAlarm ? "НЕ ХВАТИТ упора при наихудшем отказе" : "Запас есть при всех отказах") + " (" + Theme.num(s.caAge, 0) + " с назад). Сила / момент, которые наберутся:"
                                      + (s.caSingle ? " · группы не заданы — одиночные отказы" : "") }
                        Repeater {
                            model: s.caGroups
                            delegate: RowLayout {
                                Layout.fillWidth: true; spacing: 8
                                Rectangle { implicitWidth: 10; implicitHeight: 10; radius: 5; color: modelData.alarm ? Theme.alarm : modelData.holds ? Theme.ok : Theme.warn }
                                Label { text: "отказ " + modelData.name + " (" + modelData.ids + ")"; color: Theme.text; Layout.fillWidth: true; font.pixelSize: Theme.fsSmall; elide: Text.ElideRight }
                                Label { text: Theme.num(modelData.force, 0) + " / " + Theme.num(modelData.moment, 0) + " %"; color: modelData.holds ? Theme.text : Theme.alarm; font.pixelSize: Theme.fsSmall; font.family: Theme.mono }
                            }
                        }
                    }
                }
                Panel {
                    Layout.fillWidth: true; Layout.preferredHeight: 190
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 10; spacing: 4
                        Caption { text: "Датчики" }
                        ListView {
                            Layout.fillWidth: true; Layout.fillHeight: true; clip: true
                            model: core.sensors
                            delegate: RowLayout {
                                width: ListView.view.width; spacing: 8
                                Rectangle { implicitWidth: 10; implicitHeight: 10; radius: 5; color: modelData.valid ? (modelData.state === "OK" || modelData.state === "" ? Theme.ok : Theme.warn) : Theme.alarm }
                                Label { text: modelData.id; color: Theme.text; Layout.preferredWidth: 80; font.pixelSize: Theme.fsSmall }
                                Label { text: modelData.kind; color: Theme.dim; Layout.preferredWidth: 90; font.pixelSize: Theme.fsSmall }
                                Label { text: modelData.valid ? (modelData.state && modelData.state !== "OK" ? modelData.state : "норма") : "нет данных"; color: modelData.valid ? Theme.text : Theme.alarm; Layout.fillWidth: true; font.pixelSize: Theme.fsSmall }
                                Label { text: modelData.age >= 0 ? Theme.num(modelData.age, 1) + " с" : "—"; color: Theme.dim; font.pixelSize: Theme.fsSmall; font.family: Theme.mono }
                            }
                        }
                    }
                }
            }
        }

        // ------------------------------------------------ движители
        Panel {
            Layout.fillWidth: true; Layout.preferredHeight: 214
            RowLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 8
                Repeater {
                    model: core.thrusters
                    delegate: ThrusterTile { t: modelData; Layout.fillWidth: true; Layout.fillHeight: true; Layout.maximumWidth: 360 }
                }
                Label { visible: core.thrusters.length === 0; text: "Движители появятся после загрузки судна"; color: Theme.dim; Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter }
                Item { Layout.fillWidth: true; visible: core.thrusters.length > 0 && core.thrusters.length < 5 }
            }
        }

        // ------------------------------------------------ журнал и команда
        Panel {
            Layout.fillWidth: true; Layout.preferredHeight: 118
            RowLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 8
                ListView {
                    Layout.fillWidth: true; Layout.fillHeight: true; clip: true
                    model: core.log
                    delegate: Label { text: modelData; color: modelData.indexOf("ТРЕВОГА") >= 0 ? Theme.warn : Theme.dim; font.pixelSize: Theme.fsSmall; font.family: Theme.mono; width: ListView.view.width; elide: Text.ElideRight }
                }
                ColumnLayout {
                    Layout.fillWidth: false; Layout.preferredWidth: 520
                    Caption { text: "Команда ядра (help — список)" }
                    TextField { id: cmd; Layout.fillWidth: true; placeholderText: "например: sim wind 15 90"; font.family: Theme.mono
                                onAccepted: { core.command(text); text = "" } }
                }
            }
        }
    }
}

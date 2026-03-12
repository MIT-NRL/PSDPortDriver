from __future__ import annotations

import datetime as dt
from collections.abc import Sequence

try:
    import epics
except Exception:  # pragma: no cover - fallback only matters in runtime environments
    epics = None

from pydm import Display
from pydm.widgets.label import PyDMLabel
from qtpy import QtCore, QtGui, QtWidgets


class _PVSource(PyDMLabel):
    def __init__(self, channel, on_value=None, on_connect=None, parent=None):
        super().__init__(parent=parent, init_channel=channel)
        self._on_value = on_value
        self._on_connect = on_connect
        self.hide()

    def value_changed(self, new_value):
        super().value_changed(new_value)
        if self._on_value is not None:
            self._on_value(new_value)

    def connection_changed(self, connected):
        super().connection_changed(connected)
        if self._on_connect is not None:
            self._on_connect(connected)


class NeuNETRegisterReadout(Display):
    _DUMP_START = 0x180
    _DUMP_END = 0x1B5
    _GROUPS = (
        ("0x180-0x187", "Register usage control area", 0x180, 8),
        ("0x188-0x18A", "Status register", 0x188, 3),
        ("0x18B-0x18F", "Pulse ID counter", 0x18B, 5),
        ("0x190-0x196", "Device time counter", 0x190, 7),
        ("0x198-0x19F", "LLD and TOF limit area", 0x198, 8),
        ("0x1A0-0x1AF", "System area", 0x1A0, 16),
        ("0x1B0-0x1B5", "Mode setting", 0x1B0, 6),
    )

    def __init__(self, parent=None, args=None, macros=None):
        super().__init__(parent=parent, args=args, macros=macros, ui_filename=None)
        self._sources = []
        self._group_raw = {}
        self._have_rendered_groups = False
        self._group_pvnames = {}
        self._startup_retry_delays_ms = (250, 1000, 2500)
        self._build_ui()
        self._populate_static_rows()
        self._connect_channels()
        self._schedule_startup_retries()

    def _build_ui(self):
        self.setWindowTitle("NeuNET Register Readout")
        self.resize(1100, 420)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("NeuNET Register Readout")
        title_font = QtGui.QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("background-color: rgb(218, 218, 218); padding: 6px;")
        layout.addWidget(title)

        info = QtWidgets.QLabel(
            "Protocol-grouped live decode of registers 0x180-0x1B5. "
            "The right column interprets selected fields from the raw dump."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        self.status_label = QtWidgets.QLabel("Waiting for raw register dump...")
        self.status_label.setStyleSheet("color: rgb(80, 80, 80);")
        layout.addWidget(self.status_label)

        self.table = QtWidgets.QTableWidget(len(self._GROUPS), 4)
        self.table.setHorizontalHeaderLabels(
            ["Address", "Description", "Value", "Translation"]
        )
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(
            0, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            1, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            2, QtWidgets.QHeaderView.ResizeToContents
        )
        self.table.horizontalHeader().setSectionResizeMode(
            3, QtWidgets.QHeaderView.Stretch
        )
        layout.addWidget(self.table)

        self._mono_font = QtGui.QFont("Monospace")
        self._mono_font.setStyleHint(QtGui.QFont.TypeWriter)
        self._mono_font.setPointSize(10)

    def _populate_static_rows(self):
        for row, (address, description, _, _) in enumerate(self._GROUPS):
            self._set_item(row, 0, address, self._mono_font)
            self._set_item(row, 1, description)
            self._set_item(row, 2, "")
            self._set_item(row, 3, "")

    def _connect_channels(self):
        prefix = str(self.macros().get("P", "") or "")
        record = str(self.macros().get("R", "") or "")
        raw_address = f"ca://{prefix}{record}RegDumpRaw_RBV"
        self.status_label.setText(f"Source: {raw_address}")
        raw_source = _PVSource(
            channel=raw_address,
            on_value=self._on_raw_dump_changed,
            on_connect=self._on_raw_connection_changed,
            parent=self,
        )
        self._sources.append(raw_source)

        for row, (_, _, address, _) in enumerate(self._GROUPS):
            line_address = f"ca://{prefix}{record}RegDumpLine{row}_RBV"
            self._group_pvnames[address] = f"{prefix}{record}RegDumpLine{row}_RBV"
            source = _PVSource(
                channel=line_address,
                on_connect=self._make_group_connection_slot(address),
                on_value=self._make_group_slot(address),
                parent=self,
            )
            self._sources.append(source)

    def _schedule_startup_retries(self):
        for delay_ms in self._startup_retry_delays_ms:
            QtCore.QTimer.singleShot(delay_ms, self._snapshot_missing_groups)

    def _make_group_slot(self, address):
        def slot(value):
            self._on_group_changed(address, value)

        return slot

    def _make_group_connection_slot(self, address):
        def slot(connected):
            if connected:
                QtCore.QTimer.singleShot(
                    50, lambda addr=address: self._snapshot_missing_groups(addr)
                )

        return slot

    def _set_item(self, row, col, text, font=None):
        item = self.table.item(row, col)
        if item is None:
            item = QtWidgets.QTableWidgetItem()
            self.table.setItem(row, col, item)
        item.setText(str(text))
        if font is not None:
            item.setFont(font)

    def _clear_values(self, message):
        self.status_label.setText(message)
        self._have_rendered_groups = False
        for row in range(len(self._GROUPS)):
            self._set_item(row, 2, "")
            self._set_item(row, 3, "")

    def _on_raw_connection_changed(self, connected):
        if connected:
            QtCore.QTimer.singleShot(50, self._snapshot_missing_groups)

    def _on_raw_dump_changed(self, value):
        text = self._coerce_text(value)
        if not text:
            return
        if text.startswith("<"):
            self._clear_values(f"Raw dump status: {text}")
            return

        raw_bytes = self._parse_raw_dump(text)
        if raw_bytes is None:
            if not self._have_rendered_groups:
                self.status_label.setText(
                    f"Waiting for grouped register PVs... raw dump parse pending ({type(value).__name__})"
                )
            return

        self.status_label.setText("Live decode from RegDumpRaw_RBV")
        self._render_dump(raw_bytes)

    def _on_group_changed(self, address, value):
        text = self._coerce_text(value)
        if not text or text.startswith("<"):
            self._group_raw.pop(address, None)
            self._render_available_groups()
            return

        parsed = self._parse_group_value(text)
        if parsed is None:
            self.status_label.setText(
                f"Group parse failed at 0x{address:03X} for {type(value).__name__}: {text[:80]}"
            )
            return
        self._group_raw[address] = parsed
        self._render_available_groups()

    def _render_available_groups(self):
        present = 0
        status_group = self._group_raw.get(0x188, [])
        time_mode_32 = len(status_group) >= 3 and bool(status_group[2] & 0x80)

        for row, (_, _, address, size) in enumerate(self._GROUPS):
            raw = self._group_raw.get(address, [])
            if len(raw) != size:
                self._set_item(row, 2, "")
                self._set_item(row, 3, "")
                continue

            present += 1
            self._set_item(row, 2, self._format_value(raw), self._mono_font)
            self._set_item(row, 3, self._translation_for(address, raw, time_mode_32))

        if present:
            self._have_rendered_groups = True
            self.status_label.setText(
                f"Live decode from grouped register PVs ({present}/{len(self._GROUPS)} groups)"
            )
        else:
            self.status_label.setText("Waiting for grouped register PVs...")

    def _snapshot_missing_groups(self, only_address=None):
        if epics is None:
            return

        targets = []
        for _, _, address, _ in self._GROUPS:
            if only_address is not None and address != only_address:
                continue
            if address not in self._group_raw:
                targets.append(address)

        if not targets and only_address is None:
            return

        loaded_any = False
        for address in targets:
            pvname = self._group_pvnames.get(address)
            if not pvname:
                continue
            try:
                value = epics.caget(pvname, as_string=True, timeout=0.5)
            except Exception:
                value = None
            text = self._coerce_text(value)
            if not text or text.startswith("<"):
                continue
            loaded_any = True
            self._on_group_changed(address, text)

        if loaded_any and not self._have_rendered_groups:
            self.status_label.setText(
                f"Waiting for grouped register PVs... ({len(self._group_raw)}/{len(self._GROUPS)} groups)"
            )

    def _parse_raw_dump(self, text):
        values = []
        for line in text.splitlines():
            payload = line.partition(":")[2] or line
            for token in payload.split():
                if len(token) == 2:
                    try:
                        values.append(int(token, 16))
                    except ValueError:
                        return None
        expected = self._DUMP_END - self._DUMP_START + 1
        if len(values) < expected:
            return None
        return values[:expected]

    def _coerce_text(self, value):
        if value is None:
            return ""
        if isinstance(value, str):
            return value.strip()
        if isinstance(value, (bytes, bytearray)):
            return bytes(value).decode("ascii", errors="ignore").replace("\x00", "").strip()
        if hasattr(value, "tolist") and not isinstance(value, Sequence):
            value = value.tolist()
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            try:
                if all(isinstance(item, int) for item in value):
                    raw = bytes(int(item) & 0xFF for item in value)
                    text = raw.decode("ascii", errors="ignore").replace("\x00", "").strip()
                    if text:
                        return text
                text = " ".join(str(item).strip() for item in value if str(item).strip())
                return text.strip()
            except Exception:
                pass
        return str(value).strip()

    def _parse_group_value(self, text):
        values = []
        for token in text.split():
            token = token.strip()
            if not token:
                continue
            if len(token) == 4:
                try:
                    values.append(int(token[:2], 16))
                    values.append(int(token[2:], 16))
                except ValueError:
                    return None
            elif len(token) == 2:
                try:
                    values.append(int(token, 16))
                except ValueError:
                    return None
            else:
                return None
        return values

    @staticmethod
    def _format_value(raw):
        parts = []
        idx = 0
        while idx + 1 < len(raw):
            parts.append(f"{raw[idx]:02X}{raw[idx + 1]:02X}")
            idx += 2
        if idx < len(raw):
            parts.append(f"{raw[idx]:02X}")
        return " ".join(parts)

    def _render_dump(self, raw_bytes):
        c_value = raw_bytes[0x18A - self._DUMP_START]
        time_mode_32 = bool(c_value & 0x80)

        for row, (_, _, address, size) in enumerate(self._GROUPS):
            offset = address - self._DUMP_START
            raw = raw_bytes[offset : offset + size]
            self._set_item(row, 2, self._format_value(raw), self._mono_font)
            self._set_item(row, 3, self._translation_for(address, raw, time_mode_32))

    def _translation_for(self, address, raw, time_mode_32):
        if address == 0x180:
            rr = raw[7]
            rr_state = "read event memory mode" if (rr & 0x80) == 0 else "event access prohibited"
            return f"RR(7)={(rr >> 7) & 1} ({rr_state})"

        if address == 0x188:
            st = (raw[0] << 8) | raw[1]
            c_value = raw[2]
            clock_state = "running" if (st & 0x80) else "stopped"
            fifo_state = "set" if (st & 0x40) else "clear"
            time_mode = "32-bit" if (c_value & 0x80) else "30-bit"
            crate = c_value & 0x7F
            return (
                f"clock={clock_state}; fifo-clear={fifo_state}; "
                f"time={time_mode}; crate={crate}"
            )

        if address == 0x18B:
            return f"pulse no.={int.from_bytes(bytes(raw), byteorder='big')}"

        if address == 0x190:
            seconds = int.from_bytes(bytes(raw[:4]), byteorder="big")
            subseconds = raw[4] / 256.0
            if not time_mode_32:
                seconds &= 0x3FFFFFFF
            timestamp = dt.datetime(2008, 1, 1) + dt.timedelta(
                seconds=seconds + subseconds
            )
            mode = "32-bit" if time_mode_32 else "30-bit approx"
            tail = self._format_value(raw[5:])
            tail_text = f"; tail={tail}" if tail else ""
            return f"{timestamp.isoformat(sep=' ', timespec='milliseconds')} ({mode}){tail_text}"

        if address == 0x198:
            lld = int.from_bytes(bytes(raw[:2]), byteorder="big")
            tmh = int.from_bytes(bytes(raw[2:5]), byteorder="big")
            tml = int.from_bytes(bytes(raw[5:8]), byteorder="big")
            return (
                f"LLD(raw)=0x{lld:04X} ({lld}); "
                f"TMH={tmh} ticks; TML={tml} ticks"
            )

        if address == 0x1A0:
            return "reserved / undocumented system area"

        if address == 0x1B0:
            resolution = raw[4]
            gate = raw[5]
            transfer = "one-way" if (gate & 0x80) else "handshake"
            t0_sync = "enabled" if (gate & 0x40) else "disabled"
            if resolution == 0x8A:
                resolution_text = "0x8A (driver 14-bit setting)"
            else:
                resolution_text = f"0x{resolution:02X} (raw)"
            return (
                f"0x1B4={resolution_text}; "
                f"0x1B5 gate=0x{gate:02X} ({transfer}, T0 sync {t0_sync})"
            )

        return ""

    def closeEvent(self, event):
        super().closeEvent(event)

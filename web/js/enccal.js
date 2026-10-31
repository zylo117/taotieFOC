// Encoder calibration table viewer (module 10a).
// Depends on: state, CUSTOM_REGS, readRegister/writeRegister, log.

        async function runCalibrationLookup(direction) {
            const isForward = direction === 'forward';
            const input = document.getElementById(isForward ? 'calibrationRawInput' : 'calibrationCorrectedInput');
            const button = document.getElementById(isForward ? 'calibrationRawLookupBtn' : 'calibrationCorrectedLookupBtn');
            const valueOutput = document.getElementById(isForward ? 'calibrationRawOutputValue' : 'calibrationCorrectedOutputValue');
            const angleOutput = document.getElementById(isForward ? 'calibrationRawOutputAngle' : 'calibrationCorrectedOutputAngle');
            const inputValue = Number(input.value);
            if (!Number.isFinite(inputValue) || input.value.trim() === '') {
                valueOutput.textContent = '--';
                angleOutput.textContent = '--';
                return;
            }
            if (state.calibrationLookupInFlight) {
                return;
            }

            const requestId = ++state.calibrationLookupRequestId;
            state.calibrationLookupInFlight = true;
            state.calibrationLookupActiveDirection = direction;
            updateLookupInputAvailability();
            valueOutput.textContent = '等待固件计算…';
            angleOutput.textContent = '--';
            button.disabled = true;
            try {
                const result = await requestFirmwareCalibrationLookup(direction, inputValue);
                if (state.calibrationLookupRequestId !== requestId) {
                    return;
                }
                valueOutput.textContent = `0x${result.toString(16).padStart(4, '0').toUpperCase()} (${result})`;
                angleOutput.textContent = `${toCalibrationAngle(result).toFixed(3)}°`;
            } catch (error) {
                if (state.calibrationLookupRequestId !== requestId) {
                    return;
                }
                valueOutput.textContent = `查询失败：${error.message}`;
                angleOutput.textContent = '--';
                log(`[校准互查] 查询失败 direction=${direction} input=${inputValue}: ${error.message}`);
            } finally {
                if (state.calibrationLookupRequestId === requestId) {
                    state.calibrationLookupInFlight = false;
                    state.calibrationLookupActiveDirection = null;
                    updateLookupInputAvailability();
                    document.getElementById('calibrationRawLookupBtn').disabled = false;
                    document.getElementById('calibrationCorrectedLookupBtn').disabled = false;
                }
            }
        }

        function renderCalibrationTablePage() {
            const kind = document.getElementById('calibrationTableSelect').value;
            const table = kind === 'main' ? state.calibrationMainTable : state.calibrationFastTable;
            const values = document.getElementById('calibrationTableValues');
            const pageInput = document.getElementById('calibrationTablePageInput');
            if (!table) {
                values.textContent = kind === 'main' ? '主表尚未读取' : '快速表尚未读取，切换到快速表后会按需加载';
                document.getElementById('calibrationTablePageStatus').textContent = '未读取';
                return;
            }
            const pageSize = 256;
            const pageCount = Math.ceil(table.length / pageSize);
            const page = clamp(Math.round(Number(pageInput.value) || 1), 1, pageCount);
            pageInput.value = String(page);
            pageInput.max = String(pageCount);
            const first = (page - 1) * pageSize;
            const last = Math.min(first + pageSize, table.length);
            const rows = [];
            for (let index = first; index < last; index++) {
                const value = table[index];
                rows.push(`${String(index).padStart(5, '0')}  0x${value.toString(16).padStart(4, '0').toUpperCase()}  ${value}`);
            }
            values.textContent = rows.join('\n');
            document.getElementById('calibrationTablePageStatus').textContent = `第 ${page}/${pageCount} 页 · ${first}-${last - 1}`;
            updateCalibrationLookupDisplays();
        }

        function exportCalibrationTable() {
            const kind = document.getElementById('calibrationTableSelect').value;
            const table = kind === 'main' ? state.calibrationMainTable : state.calibrationFastTable;
            if (!table) {
                log('[WARN] 当前表尚未读取完成，无法导出');
                return;
            }
            const csv = `index,value,hex\n${Array.from(table, (value, index) => `${index},${value},0x${value.toString(16).padStart(4, '0').toUpperCase()}`).join('\n')}\n`;
            const url = URL.createObjectURL(new Blob([csv], {type: 'text/csv;charset=utf-8'}));
            const anchor = document.createElement('a');
            anchor.href = url;
            anchor.download = `encoder-${kind}-calibration.csv`;
            anchor.click();
            URL.revokeObjectURL(url);
        }

        async function openCalibrationTableViewer() {
            if (!state.connected) {
                log('[WARN] 尚未连接串口，无法读取校准表');
                return;
            }
            const dialog = document.getElementById('calibrationTableDialog');
            if (!dialog.open) dialog.showModal();
            state.calibrationMainTable = null;
            state.calibrationFastTable = null;
            state.calibrationFastPagesLoaded = new Uint8Array(256);
            state.calibrationTableChecksum = null;
            document.getElementById('calibrationTableChecksum').textContent = '未读取';
            document.getElementById('calibrationTableSelect').value = 'main';
            document.getElementById('calibrationTablePageInput').value = '1';
            document.getElementById('calibrationTableValues').textContent = '';
            await requestCalibrationTableDump('main');
        }

        async function closeCalibrationTableViewer() {
            if (state.calibrationDumpKind && state.connected) {
                await writeRegister(CUSTOM_REGS.CALIBRATION_TABLE_DUMP, 3);
            }
            state.calibrationDumpKind = null;
            document.getElementById('calibrationTableDialog').close();
        }


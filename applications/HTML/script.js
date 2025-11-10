document.addEventListener('DOMContentLoaded', () => {
    const websocket = new WebSocket('ws://localhost:8765');

    // --- DOM Elements ---
    const thermometerFill = document.getElementById('temperature-fill');
    const temperatureLabel = document.getElementById('temperature-label');
    const targetMarker = document.getElementById('target-marker');
    const targetLabel = document.getElementById('target-label');
    const scaleMax = document.getElementById('scale-max');
    const scaleUpper = document.getElementById('scale-upper');
    const scaleLower = document.getElementById('scale-lower');
    const scaleMin = document.getElementById('scale-min');
    const connectionStatus = document.getElementById('connection-status');

    const targetTemperatureInput = document.getElementById('target_temperature_input');
    const kpInput = document.getElementById('kp_input');
    const kiInput = document.getElementById('ki_input');
    const kdInput = document.getElementById('kd_input');
    const setTargetBtn = document.getElementById('set_target_btn');
    const setPidBtn = document.getElementById('set_pid_btn');
    const ffTableBody = document.getElementById('ff_table_body');

    // --- State ---
    let isFirstMessage = true;
    let minTempRange = 0;
    let maxTempRange = 80;
    let pendingData = null;
    let framePending = false;

    const formatters = {
        current_temperature: (v) => `${v.toFixed(2)} °C`,
        target_temperature: (v) => `${v.toFixed(2)} °C`,
        fan_speed_percent: (v) => `${v.toFixed(1)} %`,
        current_humidity: (v) => `${v.toFixed(1)} %`,
        env_temperature: (v) => `${v.toFixed(2)} °C`,
        fan_speed: (v) => v.toFixed(4),
        feedforward_speed: (v) => v.toFixed(4),
        pid_output: (v) => v.toFixed(4),
        pid_kp: (v) => v.toFixed(6),
        pid_ki: (v) => v.toFixed(6),
        pid_kd: (v) => v.toFixed(6),
        integral_error: (v) => v.toFixed(4),
        previous_error: (v) => v.toFixed(4)
    };

    formatters.current_humidity = (v) => `${v.toFixed(1)} %`;
    formatters.env_temperature = (v) => `${v.toFixed(2)} °C`;

    const ffTableInitial = [
        { temperature: 20.0, baseSpeed: 0.00 },
        { temperature: 30.0, baseSpeed: 0.10 },
        { temperature: 40.0, baseSpeed: 0.20 },
        { temperature: 50.0, baseSpeed: 0.35 },
        { temperature: 60.0, baseSpeed: 0.50 }
    ];

    // --- Utility Functions ---
    const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

    // Text update helper (no visual highlight)
    function setText(el, newText) {
        if (!el) return;
        el.textContent = newText;
    }

    function ensureRange(values) {
        let updated = false;
        values.forEach((v) => {
            if (!Number.isFinite(v)) {
                return;
            }
            if (v < minTempRange + 1) {
                minTempRange = Math.floor(v / 5) * 5 - 5;
                updated = true;
            }
            if (v > maxTempRange - 1) {
                maxTempRange = Math.ceil(v / 5) * 5 + 5;
                updated = true;
            }
        });

        if (minTempRange >= maxTempRange) {
            maxTempRange = minTempRange + 10;
            updated = true;
        }

        if (updated) {
            updateScaleLabels();
        }
    }

    function updateScaleLabels() {
        if (!scaleMax || !scaleUpper || !scaleLower || !scaleMin) {
            return;
        }
        const span = maxTempRange - minTempRange;
        const upper = minTempRange + span * 0.66;
        const lower = minTempRange + span * 0.33;

        scaleMax.textContent = `${Math.round(maxTempRange)}°C`;
        scaleUpper.textContent = `${Math.round(upper)}°C`;
        scaleLower.textContent = `${Math.round(lower)}°C`;
        scaleMin.textContent = `${Math.round(minTempRange)}°C`;
    }

    function updateThermometer(data) {
        const current = Number(data.current_temperature);
        const target = Number(data.target_temperature);

        ensureRange([current, target]);

        const range = Math.max(maxTempRange - minTempRange, 1);

        if (Number.isFinite(current)) {
            const ratio = clamp((current - minTempRange) / range, 0, 1);
            const percent = ratio * 100;
            thermometerFill.style.height = `${percent}%`;
            const labelPercent = clamp(percent, 6, 96);
            temperatureLabel.style.bottom = `${labelPercent}%`;
            temperatureLabel.textContent = `${current.toFixed(2)} °C`;
        }

        if (Number.isFinite(target)) {
            const ratio = clamp((target - minTempRange) / range, 0, 1);
            const percent = ratio * 100;
            const clampedPercent = clamp(percent, 0, 100);
            targetMarker.style.bottom = `${clampedPercent}%`;
            const labelPercent = clamp(clampedPercent, 6, 96);
            targetLabel.style.bottom = `${labelPercent}%`;
            targetLabel.textContent = `Target ${target.toFixed(1)} °C`;
        }
    }

    function initializeControlPanel(data) {
        if (Number.isFinite(data.target_temperature)) {
            targetTemperatureInput.value = data.target_temperature.toFixed(1);
        }
        if (Number.isFinite(data.pid_kp)) {
            kpInput.value = data.pid_kp.toFixed(6);
        }
        if (Number.isFinite(data.pid_ki)) {
            kiInput.value = data.pid_ki.toFixed(6);
        }
        if (Number.isFinite(data.pid_kd)) {
            kdInput.value = data.pid_kd.toFixed(6);
        }
        updateScaleLabels();
    }

    function updateDashboard(data) {
        Object.entries(data).forEach(([key, value]) => {
            const element = document.getElementById(key);
            if (!element) return;

            let text;
            if (typeof value === 'number') {
                if (formatters[key]) {
                    text = formatters[key](value);
                } else if (Number.isInteger(value)) {
                    text = value.toString();
                } else {
                    text = value.toFixed(2);
                }
            } else {
                text = String(value);
            }

            setText(element, text);
        });
    }

    function renderFeedforwardTable() {
        ffTableBody.innerHTML = '';
        ffTableInitial.forEach((entry, index) => {
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${index}</td>
                <td><input type="number" class="control-input" value="${entry.temperature.toFixed(1)}" id="ff-h-${index}" step="0.1"></td>
                <td><input type="number" class="control-input" value="${entry.baseSpeed.toFixed(4)}" id="ff-s-${index}" step="0.01"></td>
                <td><button class="control-button ff-update-btn" data-index="${index}">Update</button></td>
            `;
            ffTableBody.appendChild(row);
        });

        document.querySelectorAll('.ff-update-btn').forEach((button) => {
            button.addEventListener('click', (event) => {
                const index = event.currentTarget.dataset.index;
                const temperature = document.getElementById(`ff-h-${index}`).value;
                const speed = document.getElementById(`ff-s-${index}`).value;
                if (temperature !== '' && speed !== '') {
                    const command = `pid_tune -ff_set ${index} ${temperature} ${speed}`;
                    console.log(`Sending command: ${command}`);
                    websocket.send(command);
                }
            });
        });
    }

    // --- WebSocket Handlers ---
    websocket.onopen = () => {
        console.log('Connected to WebSocket server');
        if (connectionStatus) {
            connectionStatus.innerHTML = '<i class="fas fa-circle connected"></i><span>Connected</span>';
            connectionStatus.className = 'connection-status connected';
        }
        renderFeedforwardTable();
    };

    websocket.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (isFirstMessage) {
                initializeControlPanel(data);
                isFirstMessage = false;
            }
            // Batch DOM updates to animation frames
            pendingData = data;
            if (!framePending) {
                framePending = true;
                requestAnimationFrame(() => {
                    try {
                        if (pendingData) {
                            updateDashboard(pendingData);
                            updateThermometer(pendingData);
                            pendingData = null;
                        }
                    } finally {
                        framePending = false;
                    }
                });
            }
        } catch (error) {
            console.error('Error parsing JSON or updating UI:', error);
        }
    };

    websocket.onclose = () => {
        console.log('Disconnected from WebSocket server');
        if (connectionStatus) {
            connectionStatus.innerHTML = '<i class="fas fa-circle disconnected"></i><span>Disconnected</span>';
            connectionStatus.className = 'connection-status disconnected';
        }
    };

    websocket.onerror = (error) => {
        console.error('WebSocket Error:', error);
        if (connectionStatus) {
            connectionStatus.innerHTML = '<i class="fas fa-circle disconnected"></i><span>Error</span>';
            connectionStatus.className = 'connection-status disconnected';
        }
    };

    // --- Event Listeners ---
    setTargetBtn.addEventListener('click', () => {
        const targetValue = targetTemperatureInput.value;
        if (targetValue !== '') {
            const command = `pid_tune -t ${targetValue}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        }
    });

    setPidBtn.addEventListener('click', () => {
        const p = kpInput.value;
        const i = kiInput.value;
        const d = kdInput.value;
        let command = 'pid_tune';

        if (p !== '') command += ` -p ${p}`;
        if (i !== '') command += ` -i ${i}`;
        if (d !== '') command += ` -d ${d}`;

        if (command !== 'pid_tune') {
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        }
    });

    // Keyboard shortcuts: press Enter to submit
    [kpInput, kiInput, kdInput].forEach((el) => {
        if (el) {
            el.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') setPidBtn.click();
            });
        }
    });
    if (targetTemperatureInput) {
        targetTemperatureInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') setTargetBtn.click();
        });
    }

    // Minimize controls: toggle widget content
    document.querySelectorAll('.minimize-btn').forEach((btn) => {
        btn.addEventListener('click', () => {
            const widget = btn.closest('.widget');
            if (widget) widget.classList.toggle('minimized');
        });
    });
});
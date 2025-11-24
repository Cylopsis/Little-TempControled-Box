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
    const pidModeSelect = document.getElementById('pid_mode_select');
    const hysteresisBandInput = document.getElementById('hysteresis_band_input');
    const setHysteresisBtn = document.getElementById('set_hysteresis_btn');
    const warmingBiasInput = document.getElementById('warming_bias_input');
    const heatingBiasInput = document.getElementById('heating_bias_input');
    const setWarmingBiasBtn = document.getElementById('set_warming_bias_btn');
    const setHeatingBiasBtn = document.getElementById('set_heating_bias_btn');
    const ffTableBody = document.getElementById('ff_table_body');
    const warmingTableBody = document.getElementById('warming_table_body');
        const lowerBoundMarker = document.getElementById('lower-bound-marker');
        const upperBoundMarker = document.getElementById('upper-bound-marker');
        const warmingLowerMarker = document.getElementById('warming-lower-marker');
        const warmingUpperMarker = document.getElementById('warming-upper-marker');
        const warmingBandEl = document.getElementById('warming-band');
    const chartContainer = document.getElementById('candlestick-chart');
    const candlePeriodEl = document.getElementById('candle_period');
    const candleOpenEl = document.getElementById('candle_open');
    const candleHighEl = document.getElementById('candle_high');
    const candleLowEl = document.getElementById('candle_low');
    const candleCloseEl = document.getElementById('candle_close');
    const footerMessageEl = document.getElementById('footer_message');

    // --- State ---
    let isFirstMessage = true;
    let minTempRange = 0;
    let maxTempRange = 80;
    const precisionBounds = { min: 0, max: 3 };
    let pricePrecision = 1;
    let pendingData = null;
    let framePending = false;
    const candleDurationMs = 60 * 1000;
    const maxCandles = 240;
    const candles = [];
    let currentCandle = null;
    let chart;
    let candleSeries;
    let priceScaleWheelBound = false;

    const formatters = {
        current_temperature: (v) => `${v.toFixed(2)} °C`,
        target_temperature: (v) => `${v.toFixed(2)} °C`,
        current_humidity: (v) => `${v.toFixed(1)} %`,
        env_temperature: (v) => `${v.toFixed(2)} °C`,
        current_ptc_temperature: (v) => `${v.toFixed(2)} °C`,
        pwm_percent: (v) => `${v.toFixed(1)} %`,
        current_pwm: (v) => v.toFixed(4),
        heat_kp: (v) => v.toFixed(6),
        heat_ki: (v) => v.toFixed(6),
        heat_kd: (v) => v.toFixed(6),
        cool_kp: (v) => v.toFixed(6),
        cool_ki: (v) => v.toFixed(6),
        warming_threshold: (v) => `${v.toFixed(3)} °C`,
        hysteresis_band: (v) => `${v.toFixed(3)} °C`,
        warming_bias: (v) => `${v.toFixed(2)} °C`,
        heating_bias: (v) => `${v.toFixed(2)} °C`
    };

    const ffTableInitial = [
        { temperature: 20.0, baseSpeed: 0.00 },
        { temperature: 30.0, baseSpeed: 0.10 },
        { temperature: 40.0, baseSpeed: 0.20 },
        { temperature: 50.0, baseSpeed: 0.35 },
        { temperature: 60.0, baseSpeed: 0.50 }
    ];

    const warmingTableInitial = [
        { target: 25.0, threshold: 3.0 },
        { target: 30.0, threshold: 2.5 },
        { target: 40.0, threshold: 2.0 },
        { target: 55.0, threshold: 1.5 },
        { target: 70.0, threshold: 1.0 }
    ];

    // --- Utility Functions ---
    const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

        const valueToPercent = (value) => {
            const span = Math.max(maxTempRange - minTempRange, 1);
            return clamp((value - minTempRange) / span, 0, 1) * 100;
        };

        const markerPercent = (value) => clamp(valueToPercent(value), 2, 98);

        function setElementDisplay(el, show) {
            if (!el) return;
            el.style.display = show ? 'block' : 'none';
        }

        function updateBoundMarker(el, value, labelText) {
            if (!el || !Number.isFinite(value)) {
                setElementDisplay(el, false);
                return;
            }
            const percent = markerPercent(value);
            el.style.bottom = `${percent}%`;
            const label = el.querySelector('.marker-label');
            if (label) label.textContent = labelText;
            setElementDisplay(el, true);
        }

        function updateWarmingBand(lowerValue, upperValue) {
            if (!warmingBandEl || !Number.isFinite(lowerValue) || !Number.isFinite(upperValue)) {
                setElementDisplay(warmingBandEl, false);
                return;
            }
            const lowerPercent = markerPercent(lowerValue);
            const upperPercent = markerPercent(upperValue);
            const bottom = Math.min(lowerPercent, upperPercent);
            const height = Math.max(Math.abs(upperPercent - lowerPercent), 1.5);
            warmingBandEl.style.bottom = `${bottom}%`;
            warmingBandEl.style.height = `${height}%`;
            setElementDisplay(warmingBandEl, true);
        }

        function hideBoundVisuals() {
            [lowerBoundMarker, upperBoundMarker, warmingLowerMarker, warmingUpperMarker].forEach((el) => setElementDisplay(el, false));
            setElementDisplay(warmingBandEl, false);
        }

    // Text update helper (no visual highlight)
    function setText(el, newText) {
        if (!el) return;
        el.textContent = newText;
    }

    function formatWithPrecision(value, precision = pricePrecision) {
        if (!Number.isFinite(value)) return '--';
        return `${value.toFixed(precision)} °C`;
    }

    function minMoveForPrecision(precision) {
        if (precision <= 0) return 1;
        return Number((10 ** -precision).toFixed(precision));
    }

    function applyPricePrecision() {
        if (chart) {
            chart.applyOptions({
                localization: {
                    priceFormatter: (price) => `${price.toFixed(pricePrecision)} °C`
                }
            });
        }
        if (candleSeries) {
            candleSeries.applyOptions({
                priceFormat: {
                    type: 'price',
                    precision: pricePrecision,
                    minMove: minMoveForPrecision(pricePrecision)
                }
            });
        }
        updateCandleSummary(currentCandle);
    }

    function setPricePrecision(nextPrecision, suppressMessage = false) {
        const clamped = Math.max(precisionBounds.min, Math.min(precisionBounds.max, nextPrecision));
        if (clamped === pricePrecision) {
            if (!suppressMessage && footerMessageEl) {
                footerMessageEl.textContent = `Scale precision locked at ${pricePrecision} decimal${pricePrecision === 1 ? '' : 's'}`;
            }
            return;
        }
        pricePrecision = clamped;
        applyPricePrecision();
        if (!suppressMessage && footerMessageEl) {
            footerMessageEl.textContent = `Scale precision: ${pricePrecision} decimal${pricePrecision === 1 ? '' : 's'}`;
        }
    }

    function handlePriceScaleWheel(event) {
        if (!chartContainer) return;
        const rect = chartContainer.getBoundingClientRect();
        const scaleZoneWidth = 80;
        if (event.clientX < rect.right - scaleZoneWidth) {
            return;
        }
        event.preventDefault();
        if (event.deltaY < 0) {
            setPricePrecision(pricePrecision + 1);
        } else if (event.deltaY > 0) {
            setPricePrecision(pricePrecision - 1);
        }
    }

    function formatPeriod(timestampMs) {
        return new Date(timestampMs).toLocaleTimeString(undefined, { hour: '2-digit', minute: '2-digit' });
    }

    function initCandlestickChart() {
        if (!chartContainer || !window.LightweightCharts) {
            return;
        }

    const { createChart, CrosshairMode } = window.LightweightCharts;
    const rect = chartContainer.getBoundingClientRect();
    const initialWidth = Math.max(rect.width || chartContainer.clientWidth || 640, 320);
    const initialHeight = Math.max(rect.height || chartContainer.clientHeight || 360, 320);
        chart = createChart(chartContainer, {
            layout: {
                background: { type: 'solid', color: 'transparent' },
                textColor: '#e2e8f0',
                fontFamily: 'Segoe UI, sans-serif'
            },
            grid: {
                vertLines: { color: 'rgba(148, 163, 184, 0.12)' },
                horzLines: { color: 'rgba(148, 163, 184, 0.12)' }
            },
            crosshair: {
                mode: CrosshairMode.Magnet,
                vertLine: { color: 'rgba(226, 232, 240, 0.3)' },
                horzLine: { color: 'rgba(226, 232, 240, 0.3)' }
            },
            rightPriceScale: {
                borderColor: 'rgba(226, 232, 240, 0.12)',
                textColor: '#f8fafc'
            },
            timeScale: {
                borderColor: 'rgba(226, 232, 240, 0.12)',
                secondsVisible: false,
                timeVisible: true,
                rightOffset: 6,
                barSpacing: 12
            },
            localization: {
                priceFormatter: (price) => `${price.toFixed(pricePrecision)} °C`
            },
            width: initialWidth,
            height: initialHeight
        });

        candleSeries = chart.addCandlestickSeries({
            upColor: '#34d399',
            downColor: '#f87171',
            borderDownColor: '#f87171',
            borderUpColor: '#34d399',
            wickDownColor: '#f87171',
            wickUpColor: '#34d399'
        });

        chart.timeScale().fitContent();

        applyPricePrecision();

        if (!priceScaleWheelBound) {
            chartContainer.addEventListener('wheel', handlePriceScaleWheel, { passive: false });
            priceScaleWheelBound = true;
        }

        const resize = () => {
            if (!chart) return;
            const width = Math.max(chartContainer.clientWidth || chartContainer.getBoundingClientRect().width, 320);
            const height = Math.max(chartContainer.clientHeight || chartContainer.getBoundingClientRect().height, 320);
            chart.applyOptions({ width, height });
        };

        window.addEventListener('resize', resize);
        if (window.ResizeObserver) {
            const observer = new ResizeObserver(resize);
            observer.observe(chartContainer);
        }
    }

    function syncCandles() {
        if (!candleSeries) return;
        const formatted = candles.map(({ time, open, high, low, close }) => ({ time, open, high, low, close }));
        candleSeries.setData(formatted);
        if (chart) {
            chart.timeScale().scrollToRealTime();
        }
    }

    function updateCandleSummary(candle) {
        if (!candle) return;
        if (candlePeriodEl) setText(candlePeriodEl, formatPeriod(candle.bucketStart));
    if (candleOpenEl) setText(candleOpenEl, formatWithPrecision(candle.open));
    if (candleHighEl) setText(candleHighEl, formatWithPrecision(candle.high));
    if (candleLowEl) setText(candleLowEl, formatWithPrecision(candle.low));
    if (candleCloseEl) setText(candleCloseEl, formatWithPrecision(candle.close));
    }

    function updateCandles(data) {
        if (!candleSeries) return;
        const temp = Number(data.current_temperature);
        if (!Number.isFinite(temp)) return;

        const now = Date.now();
        const bucketStart = Math.floor(now / candleDurationMs) * candleDurationMs;
        const bucketTime = Math.floor(bucketStart / 1000);

        if (!currentCandle || currentCandle.bucketStart !== bucketStart) {
            currentCandle = {
                bucketStart,
                time: bucketTime,
                open: temp,
                high: temp,
                low: temp,
                close: temp
            };
            candles.push(currentCandle);
            if (candles.length > maxCandles) {
                candles.splice(0, candles.length - maxCandles);
            }
            syncCandles();
        } else {
            currentCandle.high = Math.max(currentCandle.high, temp);
            currentCandle.low = Math.min(currentCandle.low, temp);
            currentCandle.close = temp;
            candleSeries.update({
                time: currentCandle.time,
                open: currentCandle.open,
                high: currentCandle.high,
                low: currentCandle.low,
                close: currentCandle.close
            });
        }

        updateCandleSummary(currentCandle);
    }

    initCandlestickChart();

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
        const warmingThreshold = Number(data.warming_threshold);
        const hysteresis = Number(data.hysteresis_band);
        const hasBounds = Number.isFinite(target) && Number.isFinite(warmingThreshold) && Number.isFinite(hysteresis);

        const rangeValues = [];
        if (Number.isFinite(current)) rangeValues.push(current);
        if (Number.isFinite(target)) rangeValues.push(target);

        let lowerBound;
        let upperBound;
        let warmingLower;
        let warmingUpper;

        if (hasBounds) {
            lowerBound = target - hysteresis - warmingThreshold;
            warmingLower = target - hysteresis / 2.0;
            warmingUpper = target + hysteresis / 2.0;
            upperBound = target + hysteresis;
            [lowerBound, warmingLower, warmingUpper, upperBound].forEach((value) => {
                if (Number.isFinite(value)) {
                    rangeValues.push(value);
                }
            });
        }

        ensureRange(rangeValues);

        if (Number.isFinite(current)) {
            const percent = valueToPercent(current);
            thermometerFill.style.height = `${percent}%`;
            const labelPercent = clamp(percent, 6, 96);
            temperatureLabel.style.bottom = `${labelPercent}%`;
            temperatureLabel.textContent = `${current.toFixed(2)} °C`;
        }

        if (Number.isFinite(target)) {
            const percent = markerPercent(target);
            targetMarker.style.bottom = `${percent}%`;
            const labelPercent = clamp(percent, 6, 96);
            targetLabel.style.bottom = `${labelPercent}%`;
            targetLabel.textContent = `Target ${target.toFixed(1)} °C`;
        }

        if (hasBounds) {
            updateBoundMarker(lowerBoundMarker, lowerBound, `Lower ${lowerBound.toFixed(1)}°C`);
            updateBoundMarker(upperBoundMarker, upperBound, `Upper ${upperBound.toFixed(1)}°C`);
            updateBoundMarker(warmingLowerMarker, warmingLower, `Warm ↓ ${warmingLower.toFixed(1)}°C`);
            updateBoundMarker(warmingUpperMarker, warmingUpper, `Warm ↑ ${warmingUpper.toFixed(1)}°C`);
            updateWarmingBand(warmingLower, warmingUpper);
        } else {
            hideBoundVisuals();
        }
    }

    function enrichTelemetry(data) {
        const enriched = { ...data };
        if (Number.isFinite(data.current_pwm)) {
            enriched.pwm_percent = data.current_pwm * 100.0;
        }
        if (enriched.current_ptc_temperature === undefined && Number.isFinite(enriched.ptc_temperature)) {
            enriched.current_ptc_temperature = enriched.ptc_temperature;
        }
        return enriched;
    }

    function initializeControlPanel(data) {
        if (Number.isFinite(data.target_temperature)) {
            targetTemperatureInput.value = data.target_temperature.toFixed(1);
        }
        if (Number.isFinite(data.heat_kp)) {
            kpInput.value = data.heat_kp.toFixed(6);
        }
        if (Number.isFinite(data.heat_ki)) {
            kiInput.value = data.heat_ki.toFixed(6);
        }
        if (Number.isFinite(data.heat_kd)) {
            kdInput.value = data.heat_kd.toFixed(6);
        }
        if (hysteresisBandInput && Number.isFinite(data.hysteresis_band)) {
            hysteresisBandInput.value = data.hysteresis_band.toFixed(3);
        }
        if (warmingBiasInput && Number.isFinite(data.warming_bias)) {
            warmingBiasInput.value = data.warming_bias.toFixed(2);
        }
        if (heatingBiasInput && Number.isFinite(data.heating_bias)) {
            heatingBiasInput.value = data.heating_bias.toFixed(2);
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
                    const command = `tune ff 0 ${temperature} ${speed}`;
                    console.log(`Sending command: ${command}`);
                    websocket.send(command);
                }
            });
        });
    }

    function renderWarmingTable() {
        if (!warmingTableBody) return;
        warmingTableBody.innerHTML = '';
        warmingTableInitial.forEach((entry, index) => {
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${index}</td>
                <td><input type="number" class="control-input" value="${entry.target.toFixed(1)}" id="warm-target-${index}" step="0.1"></td>
                <td><input type="number" class="control-input" value="${entry.threshold.toFixed(3)}" id="warm-threshold-${index}" step="0.1"></td>
                <td><button class="control-button warm-update-btn" data-index="${index}">Update</button></td>
            `;
            warmingTableBody.appendChild(row);
        });

        document.querySelectorAll('.warm-update-btn').forEach((button) => {
            button.addEventListener('click', (event) => {
                const index = event.currentTarget.dataset.index;
                const targetInput = document.getElementById(`warm-target-${index}`);
                const thresholdInput = document.getElementById(`warm-threshold-${index}`);
                if (!targetInput || !thresholdInput) return;
                const targetValue = targetInput.value;
                const thresholdValue = thresholdInput.value;
                if (targetValue !== '' && thresholdValue !== '') {
                    const command = `tune ff 1 ${targetValue} ${thresholdValue}`;
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
        renderWarmingTable();
        if (footerMessageEl) {
            footerMessageEl.textContent = 'Connected to data stream';
        }
    };

    websocket.onmessage = (event) => {
        try {
            const raw = JSON.parse(event.data);
            const data = enrichTelemetry(raw);
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
                            updateCandles(pendingData);
                            if (footerMessageEl) {
                                footerMessageEl.textContent = `Last update: ${new Date().toLocaleTimeString()}`;
                            }
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
        if (footerMessageEl) {
            footerMessageEl.textContent = 'Connection closed';
        }
    };

    websocket.onerror = (error) => {
        console.error('WebSocket Error:', error);
        if (connectionStatus) {
            connectionStatus.innerHTML = '<i class="fas fa-circle disconnected"></i><span>Error</span>';
            connectionStatus.className = 'connection-status disconnected';
        }
        if (footerMessageEl) {
            footerMessageEl.textContent = 'Connection error';
        }
    };

    // --- Event Listeners ---
    setTargetBtn.addEventListener('click', () => {
        const targetValue = targetTemperatureInput.value;
        if (targetValue !== '') {
            const command = `tune target ${targetValue}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        }
    });

    setPidBtn.addEventListener('click', () => {
        const mode = pidModeSelect ? pidModeSelect.value : 'heat';
        const params = [];
        const p = kpInput.value;
        const i = kiInput.value;
        const d = kdInput.value;
        if (p !== '') params.push({ param: 'kp', value: p });
        if (i !== '') params.push({ param: 'ki', value: i });
        if (d !== '') params.push({ param: 'kd', value: d });

        const filtered = params.filter(({ param }) => !(mode === 'cool' && param === 'kd'));
        if (filtered.length === 0) {
            if (mode === 'cool' && params.some(({ param }) => param === 'kd')) {
                console.warn('Cooling PI does not support Kd. Ignoring.');
            }
            return;
        }

        filtered.forEach(({ param, value }) => {
            const command = `tune ${mode} ${param} ${value}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        });
    });

    function syncKdInputState() {
        if (!pidModeSelect || !kdInput) {
            return;
        }
        const coolingMode = pidModeSelect.value === 'cool';
        kdInput.disabled = coolingMode;
        kdInput.placeholder = coolingMode ? 'N/A for cooling PI' : '';
        if (coolingMode) {
            kdInput.value = '';
        }
    }

    if (pidModeSelect) {
        pidModeSelect.addEventListener('change', syncKdInputState);
        syncKdInputState();
    }

    if (setHysteresisBtn) {
        setHysteresisBtn.addEventListener('click', () => {
            if (!hysteresisBandInput) return;
            const value = hysteresisBandInput.value;
            if (value === '') return;
            const command = `tune hys ${value}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        });
    }

    if (setWarmingBiasBtn) {
        setWarmingBiasBtn.addEventListener('click', () => {
            if (!warmingBiasInput) return;
            const value = warmingBiasInput.value;
            if (value === '') return;
            const command = `tune warmbias ${value}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        });
    }

    if (setHeatingBiasBtn) {
        setHeatingBiasBtn.addEventListener('click', () => {
            if (!heatingBiasInput) return;
            const value = heatingBiasInput.value;
            if (value === '') return;
            const command = `tune heatbias ${value}`;
            console.log(`Sending command: ${command}`);
            websocket.send(command);
        });
    }

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

    if (hysteresisBandInput && setHysteresisBtn) {
        hysteresisBandInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') setHysteresisBtn.click();
        });
    }

    if (warmingBiasInput && setWarmingBiasBtn) {
        warmingBiasInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') setWarmingBiasBtn.click();
        });
    }

    if (heatingBiasInput && setHeatingBiasBtn) {
        heatingBiasInput.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') setHeatingBiasBtn.click();
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
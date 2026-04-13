let chart = null;
let shooterData = null;
let currentDistance = 0;
let isConnected = false;
let distanceUpdateInterval = null;

const RPM_SNAP = 25;
const ANGLE_SNAP = 1;
const DISTANCE_SNAP = 0.01;

let undoStack = [];
const MAX_UNDO_STACK = 50;

let dragState = {
    isDragging: false,
    datasetIndex: null,
    pointIndex: null,
    startX: null,
    startY: null
};

function saveState() {
    const state = JSON.parse(JSON.stringify(shooterData));
    undoStack.push(state);
    if (undoStack.length > MAX_UNDO_STACK) {
        undoStack.shift();
    }
    updateUndoButton();
}

function undo() {
    if (undoStack.length === 0) return;
    
    const previousState = undoStack.pop();
    shooterData = previousState;
    
    updateChart();
    updatePointCount();
    updateUndoButton();
    
    saveAllPointsToServer();
}

function updateUndoButton() {
    const undoBtn = document.getElementById('undoBtn');
    undoBtn.disabled = undoStack.length === 0;
}

async function saveAllPointsToServer() {
    try {
        for (const point of shooterData.rpmPoints) {
            await fetch('/api/update-rpm-point', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    distance: point.distance,
                    rpm: point.rpm
                })
            });
        }
        for (const point of shooterData.anglePoints) {
            await fetch('/api/update-angle-point', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    distance: point.distance,
                    angle: point.angle
                })
            });
        }
        showNotification('Changes restored', 'success');
    } catch (error) {
        console.error('Error saving state:', error);
        showNotification('Failed to restore changes', 'error');
    }
}

async function init() {
    try {
        await loadShooterData();
        createChart();
        setupEventListeners();
        setupCanvasListeners();
        startDistancePolling();
        updateConnectionStatus(true);
    } catch (error) {
        console.error('Initialization error:', error);
        showNotification('Failed to connect to robot', 'error');
        updateConnectionStatus(false);
        setTimeout(init, 5000);
    }
}

async function loadShooterData() {
    const response = await fetch('/api/shooter-data');
    if (!response.ok) throw new Error('Failed to load shooter data');
    const data = await response.json();
    
    shooterData = {
        rpmPoints: data.rpmPoints || [],
        anglePoints: data.anglePoints || []
    };
    
    shooterData.rpmPoints.sort((a, b) => a.distance - b.distance);
    shooterData.anglePoints.sort((a, b) => a.distance - b.distance);
    
    updatePointCount();
}

function updateChart() {
    if (!chart) return;
    
    chart.data.datasets[0].data = shooterData.rpmPoints.map(p => ({ x: p.distance, y: p.rpm }));
    chart.data.datasets[1].data = shooterData.anglePoints.map(p => ({ x: p.distance, y: p.angle }));
    chart.update('none');
}

function createChart() {
    const ctx = document.getElementById('shooterChart').getContext('2d');
    
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                {
                    label: 'RPM',
                    data: shooterData.rpmPoints.map(p => ({ x: p.distance, y: p.rpm })),
                    borderColor: '#00d4ff',
                    backgroundColor: 'rgba(0, 212, 255, 0.1)',
                    pointBackgroundColor: '#00d4ff',
                    pointBorderColor: '#00d4ff',
                    pointRadius: 6,
                    pointHoverRadius: 8,
                    pointBorderWidth: 2,
                    yAxisID: 'y',
                    tension: 0.4,
                    borderWidth: 2
                },
                {
                    label: 'ANGLE',
                    data: shooterData.anglePoints.map(p => ({ x: p.distance, y: p.angle })),
                    borderColor: '#ffff00',
                    backgroundColor: 'rgba(255, 255, 0, 0.1)',
                    pointBackgroundColor: '#ffff00',
                    pointBorderColor: '#ffff00',
                    pointRadius: 6,
                    pointHoverRadius: 8,
                    pointBorderWidth: 2,
                    yAxisID: 'y1',
                    tension: 0.4,
                    borderWidth: 2
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                mode: 'nearest',
                axis: 'xy',
                intersect: false
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top',
                    labels: {
                        color: '#ffffff',
                        font: {
                            size: 11,
                            weight: '600',
                            family: 'monospace'
                        },
                        padding: 15,
                        usePointStyle: false,
                        boxWidth: 30,
                        boxHeight: 2
                    }
                },
                tooltip: {
                    enabled: true,
                    backgroundColor: 'rgba(0, 0, 0, 0.9)',
                    titleColor: '#ffffff',
                    bodyColor: '#00d4ff',
                    borderColor: '#00d4ff',
                    borderWidth: 1,
                    padding: 10,
                    displayColors: false,
                    titleFont: {
                        size: 11,
                        family: 'monospace'
                    },
                    bodyFont: {
                        size: 12,
                        family: 'monospace',
                        weight: 'bold'
                    },
                    callbacks: {
                        title: (items) => `${items[0].parsed.x.toFixed(2)}m`,
                        label: (item) => {
                            if (item.datasetIndex === 0) {
                                return `${item.parsed.y.toFixed(0)} RPM`;
                            } else {
                                return `${item.parsed.y.toFixed(1)}°`;
                            }
                        }
                    }
                }
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'DISTANCE (m)',
                        color: '#666666',
                        font: {
                            size: 11,
                            weight: '600',
                            family: 'monospace'
                        }
                    },
                    grid: {
                        color: 'rgba(255, 255, 255, 0.05)',
                        lineWidth: 1
                    },
                    ticks: {
                        color: '#666666',
                        font: {
                            size: 10,
                            family: 'monospace'
                        }
                    }
                },
                y: {
                    type: 'linear',
                    position: 'left',
                    title: {
                        display: true,
                        text: 'RPM',
                        color: '#00d4ff',
                        font: {
                            size: 11,
                            weight: '600',
                            family: 'monospace'
                        }
                    },
                    grid: {
                        color: 'rgba(0, 212, 255, 0.1)',
                        lineWidth: 1
                    },
                    ticks: {
                        color: '#00d4ff',
                        font: {
                            size: 10,
                            family: 'monospace'
                        }
                    }
                },
                y1: {
                    type: 'linear',
                    position: 'right',
                    title: {
                        display: true,
                        text: 'ANGLE (°)',
                        color: '#ffff00',
                        font: {
                            size: 11,
                            weight: '600',
                            family: 'monospace'
                        }
                    },
                    grid: {
                        drawOnChartArea: false
                    },
                    ticks: {
                        color: '#ffff00',
                        font: {
                            size: 10,
                            family: 'monospace'
                        }
                    }
                }
            }
        },
        plugins: [{
            id: 'currentDistanceLine',
            afterDatasetsDraw: (chart) => {
                if (currentDistance > 0) {
                    const ctx = chart.ctx;
                    const xScale = chart.scales.x;
                    const yScale = chart.scales.y;
                    
                    const x = xScale.getPixelForValue(currentDistance);
                    const topY = yScale.top;
                    const bottomY = yScale.bottom;
                    
                    ctx.save();
                    ctx.strokeStyle = '#00ff88';
                    ctx.lineWidth = 2;
                    ctx.shadowBlur = 10;
                    ctx.shadowColor = '#00ff88';
                    ctx.setLineDash([5, 5]);
                    ctx.beginPath();
                    ctx.moveTo(x, topY);
                    ctx.lineTo(x, bottomY);
                    ctx.stroke();
                    
                    ctx.shadowBlur = 0;
                    ctx.fillStyle = '#00ff88';
                    ctx.font = 'bold 11px monospace';
                    ctx.textAlign = 'center';
                    ctx.fillText(`${currentDistance.toFixed(2)}m`, x, topY - 8);
                    ctx.restore();
                }
            }
        }]
    });
    
    chart.canvas.addEventListener('mousemove', (event) => {
        if (dragState.isDragging) return;
        
        const rect = chart.canvas.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;
        
        const xScale = chart.scales.x;
        const dataX = xScale.getValueForPixel(x);
        
        const clickedDataset = detectLineClick(x, y, dataX);
        
        if (clickedDataset !== null) {
            chart.canvas.style.cursor = 'copy';
        } else {
            const elements = chart.getElementsAtEventForMode(event, 'nearest', { intersect: true }, false);
            chart.canvas.style.cursor = elements.length > 0 ? 'grab' : 'default';
        }
    });
}

function setupCanvasListeners() {
    const canvas = document.getElementById('shooterChart');
    
    canvas.addEventListener('mousedown', handleMouseDown);
    canvas.addEventListener('mousemove', handleMouseMove);
    canvas.addEventListener('mouseup', handleMouseUp);
    canvas.addEventListener('mouseleave', handleMouseUp);
    canvas.addEventListener('click', handleCanvasClick);
}

function handleMouseDown(event) {
    const rect = chart.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    const elements = chart.getElementsAtEventForMode(event, 'nearest', { intersect: true }, false);
    
    if (elements.length > 0) {
        const element = elements[0];
        dragState.isDragging = true;
        dragState.datasetIndex = element.datasetIndex;
        dragState.pointIndex = element.index;
        dragState.startX = x;
        dragState.startY = y;
        
        saveState();
        
        chart.canvas.style.cursor = 'grabbing';
        event.preventDefault();
    }
}

function handleMouseMove(event) {
    if (!dragState.isDragging) {
        return;
    }
    
    const rect = chart.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    const xScale = chart.scales.x;
    const yScale = dragState.datasetIndex === 0 ? chart.scales.y : chart.scales.y1;
    
    const dataX = xScale.getValueForPixel(x);
    const dataY = yScale.getValueForPixel(y);
    
    const snappedX = Math.max(0.1, Math.round(dataX / DISTANCE_SNAP) * DISTANCE_SNAP);
    const snappedY = dragState.datasetIndex === 0 
        ? Math.round(dataY / RPM_SNAP) * RPM_SNAP
        : Math.round(dataY / ANGLE_SNAP) * ANGLE_SNAP;
    
    if (dragState.datasetIndex === 0) {
        shooterData.rpmPoints[dragState.pointIndex] = {
            distance: snappedX,
            rpm: snappedY
        };
    } else {
        shooterData.anglePoints[dragState.pointIndex] = {
            distance: snappedX,
            angle: snappedY
        };
    }
    
    updateChart();
    event.preventDefault();
}

async function handleMouseUp(event) {
    if (!dragState.isDragging) return;
    
    if (dragState.datasetIndex === 0) {
        shooterData.rpmPoints.sort((a, b) => a.distance - b.distance);
    } else {
        shooterData.anglePoints.sort((a, b) => a.distance - b.distance);
    }
    
    updateChart();
    updatePointCount();
    
    const point = dragState.datasetIndex === 0 
        ? shooterData.rpmPoints[dragState.pointIndex]
        : shooterData.anglePoints[dragState.pointIndex];
    
    if (point) {
        if (dragState.datasetIndex === 0) {
            await updateRPMPoint(point.distance, point.rpm);
        } else {
            await updateAnglePoint(point.distance, point.angle);
        }
    }
    
    dragState.isDragging = false;
    dragState.datasetIndex = null;
    dragState.pointIndex = null;
    chart.canvas.style.cursor = 'default';
    
    event.preventDefault();
}

function handleCanvasClick(event) {
    if (dragState.isDragging) return;
    
    const elements = chart.getElementsAtEventForMode(event, 'nearest', { intersect: true }, false);
    
    if (elements.length > 0) {
        return;
    }
    
    const rect = chart.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;
    const yScale1 = chart.scales.y1;
    
    const dataX = xScale.getValueForPixel(x);
    const dataY = yScale.getValueForPixel(y);
    const dataY1 = yScale1.getValueForPixel(y);
    
    if (dataX < 0.1 || dataX > 10) return;
    
    const clickedDataset = detectLineClick(x, y, dataX);
    
    if (clickedDataset === null) {
        return;
    }
    
    saveState();
    
    const newDistance = Math.round(dataX / DISTANCE_SNAP) * DISTANCE_SNAP;
    
    if (clickedDataset === 0) {
        const newRPM = Math.round(dataY / RPM_SNAP) * RPM_SNAP;
        shooterData.rpmPoints.push({ distance: newDistance, rpm: newRPM });
        shooterData.rpmPoints.sort((a, b) => a.distance - b.distance);
        updateRPMPoint(newDistance, newRPM);
    } else {
        const newAngle = Math.round(dataY1 / ANGLE_SNAP) * ANGLE_SNAP;
        shooterData.anglePoints.push({ distance: newDistance, angle: newAngle });
        shooterData.anglePoints.sort((a, b) => a.distance - b.distance);
        updateAnglePoint(newDistance, newAngle);
    }
    
    updateChart();
    updatePointCount();
    showNotification('Point added', 'success');
}

function detectLineClick(pixelX, pixelY, dataX) {
    const threshold = 15;
    
    const interpolatedRPM = interpolateValue(dataX, shooterData.rpmPoints, 'rpm');
    const interpolatedAngle = interpolateValue(dataX, shooterData.anglePoints, 'angle');
    
    const rpmPixelY = chart.scales.y.getPixelForValue(interpolatedRPM);
    const anglePixelY = chart.scales.y1.getPixelForValue(interpolatedAngle);
    
    const distToRPM = Math.abs(pixelY - rpmPixelY);
    const distToAngle = Math.abs(pixelY - anglePixelY);
    
    if (distToRPM < threshold && distToRPM < distToAngle) {
        return 0;
    } else if (distToAngle < threshold) {
        return 1;
    }
    
    return null;
}

function interpolateValue(distance, points, property) {
    if (points.length === 0) return 0;
    if (points.length === 1) return points[0][property];
    
    const sortedPoints = [...points].sort((a, b) => a.distance - b.distance);
    
    if (distance <= sortedPoints[0].distance) {
        return sortedPoints[0][property];
    }
    
    if (distance >= sortedPoints[sortedPoints.length - 1].distance) {
        return sortedPoints[sortedPoints.length - 1][property];
    }
    
    for (let i = 0; i < sortedPoints.length - 1; i++) {
        const p1 = sortedPoints[i];
        const p2 = sortedPoints[i + 1];
        
        if (distance >= p1.distance && distance <= p2.distance) {
            const t = (distance - p1.distance) / (p2.distance - p1.distance);
            return p1[property] + t * (p2[property] - p1[property]);
        }
    }
    
    return sortedPoints[sortedPoints.length - 1][property];
}

async function updateRPMPoint(distance, rpm) {
    try {
        const response = await fetch('/api/update-rpm-point', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ distance, rpm })
        });
        
        if (!response.ok) throw new Error('Failed to update RPM point');
    } catch (error) {
        console.error('Error updating RPM point:', error);
        showNotification('Update failed', 'error');
    }
}

async function updateAnglePoint(distance, angle) {
    try {
        const response = await fetch('/api/update-angle-point', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ distance, angle })
        });
        
        if (!response.ok) throw new Error('Failed to update angle point');
    } catch (error) {
        console.error('Error updating angle point:', error);
        showNotification('Update failed', 'error');
    }
}

function setupEventListeners() {
    const liveToggle = document.getElementById('liveToggle');
    liveToggle.addEventListener('change', async (e) => {
        try {
            const response = await fetch('/api/toggle-live', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ enabled: e.target.checked })
            });
            
            if (!response.ok) throw new Error('Failed to toggle live updates');
            
            showNotification(
                e.target.checked ? 'Live mode ON' : 'Live mode OFF',
                'success'
            );
        } catch (error) {
            console.error('Error toggling live updates:', error);
            showNotification('Toggle failed', 'error');
            e.target.checked = !e.target.checked;
        }
    });
    
    const undoBtn = document.getElementById('undoBtn');
    undoBtn.addEventListener('click', undo);
    
    document.addEventListener('keydown', (e) => {
        if ((e.ctrlKey || e.metaKey) && e.key === 'z') {
            e.preventDefault();
            undo();
        }
    });
}

function startDistancePolling() {
    if (distanceUpdateInterval) {
        clearInterval(distanceUpdateInterval);
    }
    
    distanceUpdateInterval = setInterval(async () => {
        try {
            const response = await fetch('/api/current-distance');
            if (!response.ok) throw new Error('Failed to fetch distance');
            
            const data = await response.json();
            currentDistance = data.distance;
            updateCurrentDistanceDisplay(currentDistance);
            
            if (chart) {
                chart.update('none');
            }
            
            if (!isConnected) {
                updateConnectionStatus(true);
            }
        } catch (error) {
            if (isConnected) {
                updateConnectionStatus(false);
            }
        }
    }, 100);
}

function updateConnectionStatus(connected) {
    isConnected = connected;
    const statusDot = document.querySelector('.status-dot');
    
    if (connected) {
        statusDot.classList.add('connected');
    } else {
        statusDot.classList.remove('connected');
    }
}

function updatePointCount() {
    const total = shooterData.rpmPoints.length + shooterData.anglePoints.length;
    document.getElementById('pointCount').textContent = total;
}

function updateCurrentDistanceDisplay(distance) {
    document.getElementById('currentDistance').textContent = distance.toFixed(2);
}

function showNotification(message, type = 'success') {
    const notification = document.createElement('div');
    notification.className = `notification ${type}`;
    notification.textContent = message;
    document.body.appendChild(notification);
    
    setTimeout(() => {
        notification.style.animation = 'slideIn 0.3s ease-out reverse';
        setTimeout(() => notification.remove(), 300);
    }, 2000);
}

document.addEventListener('DOMContentLoaded', init);

let chart = null;
let shooterData = null;
let currentDistance = 0;
let isConnected = false;
let distanceUpdateInterval = null;

const RPM_SNAP = 25;
const ANGLE_SNAP = 1;

async function init() {
    try {
        await loadShooterData();
        createChart();
        setupEventListeners();
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
    shooterData = await response.json();
    updatePointCount();
}

function createChart() {
    const ctx = document.getElementById('shooterChart').getContext('2d');
    
    const distances = shooterData.points.map(p => p.distance);
    const rpms = shooterData.points.map(p => p.flywheelRPM);
    const angles = shooterData.points.map(p => p.hoodAngleDegrees);
    
    chart = new Chart(ctx, {
        type: 'line',
        data: {
            datasets: [
                {
                    label: 'Flywheel RPM',
                    data: distances.map((d, i) => ({ x: d, y: rpms[i] })),
                    borderColor: '#4a9eff',
                    backgroundColor: 'rgba(74, 158, 255, 0.2)',
                    pointBackgroundColor: '#4a9eff',
                    pointBorderColor: '#fff',
                    pointRadius: 8,
                    pointHoverRadius: 10,
                    yAxisID: 'y',
                    tension: 0.4,
                    borderWidth: 3
                },
                {
                    label: 'Hood Angle (°)',
                    data: distances.map((d, i) => ({ x: d, y: angles[i] })),
                    borderColor: '#ffd93d',
                    backgroundColor: 'rgba(255, 217, 61, 0.2)',
                    pointBackgroundColor: '#ffd93d',
                    pointBorderColor: '#fff',
                    pointRadius: 8,
                    pointHoverRadius: 10,
                    yAxisID: 'y1',
                    tension: 0.4,
                    borderWidth: 3
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: true,
            aspectRatio: 2,
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
                        color: '#eaeaea',
                        font: {
                            size: 14,
                            weight: 'bold'
                        },
                        padding: 20,
                        usePointStyle: true
                    }
                },
                tooltip: {
                    backgroundColor: 'rgba(22, 33, 62, 0.95)',
                    titleColor: '#eaeaea',
                    bodyColor: '#eaeaea',
                    borderColor: '#2d3748',
                    borderWidth: 1,
                    padding: 12,
                    displayColors: true,
                    callbacks: {
                        title: (items) => `Distance: ${items[0].parsed.x.toFixed(2)}m`,
                        label: (item) => {
                            if (item.datasetIndex === 0) {
                                return `RPM: ${item.parsed.y.toFixed(0)}`;
                            } else {
                                return `Hood: ${item.parsed.y.toFixed(1)}°`;
                            }
                        }
                    }
                },
                dragData: {
                    round: 0,
                    showTooltip: true,
                    onDragStart: function(e, datasetIndex, index, value) {
                        return true;
                    },
                    onDrag: function(e, datasetIndex, index, value) {
                        if (datasetIndex === 0) {
                            value.y = Math.round(value.y / RPM_SNAP) * RPM_SNAP;
                        } else if (datasetIndex === 1) {
                            value.y = Math.round(value.y / ANGLE_SNAP) * ANGLE_SNAP;
                        }
                        value.x = shooterData.points[index].distance;
                        return value;
                    },
                    onDragEnd: async function(e, datasetIndex, index, value) {
                        const distance = shooterData.points[index].distance;
                        let hoodAngle, rpm;
                        
                        if (datasetIndex === 0) {
                            rpm = Math.round(value.y / RPM_SNAP) * RPM_SNAP;
                            hoodAngle = shooterData.points[index].hoodAngleDegrees;
                        } else {
                            hoodAngle = Math.round(value.y / ANGLE_SNAP) * ANGLE_SNAP;
                            rpm = shooterData.points[index].flywheelRPM;
                        }
                        
                        shooterData.points[index].flywheelRPM = rpm;
                        shooterData.points[index].hoodAngleDegrees = hoodAngle;
                        
                        chart.data.datasets[0].data[index].y = rpm;
                        chart.data.datasets[1].data[index].y = hoodAngle;
                        chart.update('none');
                        
                        await updatePoint(distance, hoodAngle, rpm);
                    }
                }
            },
            scales: {
                x: {
                    type: 'linear',
                    position: 'bottom',
                    title: {
                        display: true,
                        text: 'Distance (meters)',
                        color: '#eaeaea',
                        font: {
                            size: 14,
                            weight: 'bold'
                        }
                    },
                    grid: {
                        color: 'rgba(255, 255, 255, 0.1)'
                    },
                    ticks: {
                        color: '#eaeaea'
                    }
                },
                y: {
                    type: 'linear',
                    position: 'left',
                    title: {
                        display: true,
                        text: 'Flywheel RPM',
                        color: '#4a9eff',
                        font: {
                            size: 14,
                            weight: 'bold'
                        }
                    },
                    grid: {
                        color: 'rgba(74, 158, 255, 0.1)'
                    },
                    ticks: {
                        color: '#4a9eff'
                    }
                },
                y1: {
                    type: 'linear',
                    position: 'right',
                    title: {
                        display: true,
                        text: 'Hood Angle (degrees)',
                        color: '#ffd93d',
                        font: {
                            size: 14,
                            weight: 'bold'
                        }
                    },
                    grid: {
                        drawOnChartArea: false
                    },
                    ticks: {
                        color: '#ffd93d'
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
                    ctx.strokeStyle = '#6bcf7f';
                    ctx.lineWidth = 3;
                    ctx.setLineDash([5, 5]);
                    ctx.beginPath();
                    ctx.moveTo(x, topY);
                    ctx.lineTo(x, bottomY);
                    ctx.stroke();
                    
                    ctx.fillStyle = '#6bcf7f';
                    ctx.font = 'bold 12px sans-serif';
                    ctx.textAlign = 'center';
                    ctx.fillText(`${currentDistance.toFixed(2)}m`, x, topY - 5);
                    ctx.restore();
                }
            }
        }]
    });
}

async function updatePoint(distance, hoodAngleDegrees, flywheelRPM) {
    try {
        const response = await fetch('/api/update-point', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ distance, hoodAngleDegrees, flywheelRPM })
        });
        
        if (!response.ok) throw new Error('Failed to update point');
        
        updateLastUpdate();
        showNotification('Point updated successfully', 'success');
    } catch (error) {
        console.error('Error updating point:', error);
        showNotification('Failed to update point', 'error');
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
                e.target.checked ? 'Live updates enabled' : 'Live updates disabled',
                'success'
            );
        } catch (error) {
            console.error('Error toggling live updates:', error);
            showNotification('Failed to toggle live updates', 'error');
            e.target.checked = !e.target.checked;
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
            console.error('Error fetching distance:', error);
            if (isConnected) {
                updateConnectionStatus(false);
            }
        }
    }, 100);
}

function updateConnectionStatus(connected) {
    isConnected = connected;
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.status-text');
    
    if (connected) {
        statusDot.classList.add('connected');
        statusText.textContent = 'Connected';
    } else {
        statusDot.classList.remove('connected');
        statusText.textContent = 'Disconnected';
    }
}

function updatePointCount() {
    document.getElementById('pointCount').textContent = shooterData.points.length;
}

function updateCurrentDistanceDisplay(distance) {
    document.getElementById('currentDistance').textContent = `${distance.toFixed(2)}m`;
}

function updateLastUpdate() {
    const now = new Date();
    const timeString = now.toLocaleTimeString();
    document.getElementById('lastUpdate').textContent = timeString;
}

function showNotification(message, type = 'success') {
    const notification = document.createElement('div');
    notification.className = `notification ${type}`;
    notification.textContent = message;
    document.body.appendChild(notification);
    
    setTimeout(() => {
        notification.style.animation = 'slideIn 0.3s ease-out reverse';
        setTimeout(() => notification.remove(), 300);
    }, 3000);
}

document.addEventListener('DOMContentLoaded', init);

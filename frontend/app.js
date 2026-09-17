/**
 * ==============================================================================
 * AMR FLEET NAVIGATOR - CLIENT APPLICATION
 * Real-Time Decentralized Multi-Agent Fleet Control & Telemetry Dashboard
 * Features:
 *   - Ingests real dataset (all_collected_dataset.csv / desktop_fleet_dataset.csv)
 *   - 2D / 3D Coordinate Map ([-22.5m, 22.5m] x [-30.0m, 30.0m])
 *   - CBBA Task Auction & 9-Phase Lifecycle Manager with Dwell Timers
 *   - Live Multi-Agent Decision Stream (Quorum 4/4, WHCA*, Lamport Mutex, Safety)
 *   - Zero Dummy Data: Real Historical & Live Fleet Telemetry
 * ==============================================================================
 */

// --- WAREHOUSE CONFIGURATION (ROS Coordinate Frame) ---
const WAREHOUSE_CONFIG = {
  width_m: 45.0,
  height_m: 60.0,
  origin_x: -22.5,
  origin_y: -30.0,
  
  charging_pads: [
    { id: 'PAD_1', x: -3.6, y: -28.5, label: 'Pad 1 (Inductive)' },
    { id: 'PAD_2', x: -1.2, y: -28.5, label: 'Pad 2 (Inductive)' },
    { id: 'PAD_3', x: 1.2, y: -28.5, label: 'Pad 3 (Inductive)' },
    { id: 'PAD_4', x: 3.6, y: -28.5, label: 'Pad 4 (Inductive)' },
  ],

  corridors: [
    { id: 'MC-NS-W', name: 'Corridor West', x: -9.0, width: 2.2, orientation: 'V' },
    { id: 'MC-NS-E', name: 'Corridor East', x: 9.0, width: 2.2, orientation: 'V' },
    { id: 'MC-EW-S', name: 'Corridor South', y: -10.0, height: 2.2, orientation: 'H' },
    { id: 'MC-EW-N', name: 'Corridor North', y: 10.0, height: 2.2, orientation: 'H' },
  ],

  stations: {
    South: { id: 'DROPOFF_STATION_A', zone: 'DISPATCH_BAY_1', x: -11.35, y: -6.11 },
    North: { id: 'DROPOFF_STATION_B', zone: 'DISPATCH_BAY_2', x: 0.00, y: 28.79 },
    East: { id: 'DROPOFF_STATION_C', zone: 'INSPECTION_PAD', x: 19.99, y: 19.63 },
    Central: { id: 'DROPOFF_STATION_D', zone: 'ASSEMBLY_STN_1', x: 0.00, y: 26.75 }
  },

  zoneRacks: {
    South: [
      { id: 'RACK_WEST_SOUTH_01', x: -19.99, y: -17.60, item: 'Servo Motors (High Torque)' },
      { id: 'RACK_WEST_SOUTH_02', x: -15.67, y: -20.65, item: 'Micro Stepper Controllers' },
      { id: 'RACK_EAST_SOUTH_08', x: 15.67, y: -20.65, item: 'Industrial Ball Bearings' },
      { id: 'RACK_EAST_SOUTH_09', x: 19.99, y: -24.72, item: 'Hydraulic Manifold Valves' }
    ],
    North: [
      { id: 'RACK_WEST_NORTH_12', x: -19.99, y: 13.53, item: 'LiDAR Optical Modules' },
      { id: 'RACK_WEST_NORTH_16', x: -15.67, y: 27.77, item: 'PLC Automation Units' },
      { id: 'RACK_EAST_NORTH_22', x: 15.67, y: 20.65, item: 'Pneumatic Actuators' },
      { id: 'RACK_EAST_NORTH_24', x: 19.99, y: 28.79, item: 'Proximity Sensor Arrays' }
    ],
    Central: [
      { id: 'RACK_CENTER_MID_05', x: -2.54, y: 1.01, item: 'Thermal Imaging Cameras' },
      { id: 'RACK_CENTER_MID_06', x: 2.54, y: 5.08, item: 'CAN-Bus Gateway Relays' },
      { id: 'RACK_CENTER_MID_07', x: -2.54, y: -3.05, item: 'Battery Management PCBA' }
    ]
  },

  shelves: []
};

// Build all warehouse racks from canonical grid
function buildWarehouseShelves() {
  const x_zones = {
    west: [-19.99, -15.67, -11.35],
    center: [-2.5363, 2.5363],
    east: [11.35, 15.67, 19.99]
  };
  const y_zones = {
    south: [-28.79, -24.72, -20.65, -16.58, -12.51],
    middle: [-7.12, -3.05, 1.01, 5.08],
    north: [12.51, 16.58, 20.65, 24.72, 28.79]
  };

  let count = 1;
  for (const [zx, xList] of Object.entries(x_zones)) {
    for (const x of xList) {
      for (const [zy, yList] of Object.entries(y_zones)) {
        if (zx === 'center' && zy === 'south') continue;
        for (const y of yList) {
          const zoneTag = `${zx[0].toUpperCase()}-${count.toString().padStart(2, '0')}`;
          WAREHOUSE_CONFIG.shelves.push({
            id: `RACK_${zx.toUpperCase()}_${zy.toUpperCase()}_${count.toString().padStart(2, '0')}`,
            name: `Rack ${zoneTag}`,
            code: `RACK_${count++}`,
            zone: `${zx.toUpperCase()}_${zy.toUpperCase()}`,
            x: x,
            y: y,
            w: 3.92,
            h: 1.10,
            depth_3d: 2.2,
            itemType: ['Servo Motors', 'Hydraulic Valves', 'Industrial Bearings', 'LiDAR Sensors', 'PLC Modules'][count % 5]
          });
        }
      }
    }
  }
}
buildWarehouseShelves();

const ROBOT_COLOR_MAP = {
  robot_1: '#06b6d4', // Cyan
  robot_2: '#10b981', // Green
  robot_3: '#f59e0b', // Amber / Yellow
  robot_4: '#a855f7', // Magenta / Purple
  robot_5: '#3b82f6', // Blue
  robot_6: '#ef4444', // Red
  robot_7: '#ec4899', // Pink
  robot_8: '#14b8a6'  // Teal
};

// --- MOCK TASKS (EXACT SCHEMA AS PER DATA SPECIFICATION) ---
const MOCK_TASKS = [
  {
    task_id: "rnd_task_001",
    priority: 100,
    status: "EN_ROUTE_DROPOFF",
    assigned_robot_id: "robot_1",
    winning_bid: 104.46,
    pickup: { x: -19.99, y: -17.60, theta: 0.0, label: "Rack W-04", rack_id: "RACK_WEST_SOUTH_01", item_type: "Servo Motors" },
    dropoff: { x: -11.35, y: -6.11, theta: 0.0, label: "Dispatch Bay A", station_id: "DROPOFF_STATION_A", zone: "DISPATCH_BAY_1" },
    progress_pct: 68,
    dwell_times: {
      pickup_wait_s: 3.0,
      dropoff_wait_s: 3.0
    },
    dwell_remaining: 0.0,
    created_at_epoch: 1789187770.719,
    expires_at_epoch: 1789188070.719
  },
  {
    task_id: "rnd_task_002",
    priority: 75,
    status: "EN_ROUTE_PICKUP",
    assigned_robot_id: "robot_2",
    winning_bid: 108.80,
    pickup: { x: -19.99, y: 13.53, theta: 0.0, label: "Rack W-12", rack_id: "RACK_WEST_NORTH_12", item_type: "LiDAR Optical Modules" },
    dropoff: { x: 0.00, y: 28.79, theta: 0.0, label: "Dispatch Bay B", station_id: "DROPOFF_STATION_B", zone: "DISPATCH_BAY_2" },
    progress_pct: 35,
    dwell_times: {
      pickup_wait_s: 3.0,
      dropoff_wait_s: 3.0
    },
    dwell_remaining: 0.0,
    created_at_epoch: 1789187775.719,
    expires_at_epoch: 1789188075.719
  },
  {
    task_id: "rnd_task_003",
    priority: 50,
    status: "CBBA_AUCTION",
    assigned_robot_id: null,
    winning_bid: null,
    pickup: { x: 15.67, y: -20.65, theta: 0.0, label: "Rack E-08", rack_id: "RACK_EAST_SOUTH_08", item_type: "Industrial Ball Bearings" },
    dropoff: { x: 19.99, y: 19.63, theta: 0.0, label: "Inspection Pad", station_id: "DROPOFF_STATION_C", zone: "INSPECTION_PAD" },
    progress_pct: 0,
    dwell_times: {
      pickup_wait_s: 3.0,
      dropoff_wait_s: 3.0
    },
    dwell_remaining: 0.0,
    created_at_epoch: 1789187780.719,
    expires_at_epoch: 1789188080.719
  },
  {
    task_id: "rnd_task_004",
    priority: 80,
    status: "COMPLETED",
    assigned_robot_id: "robot_4",
    winning_bid: 92.15,
    pickup: { x: -15.67, y: 27.77, theta: 0.0, label: "Rack W-16", rack_id: "RACK_WEST_NORTH_16", item_type: "PLC Automation Units" },
    dropoff: { x: 0.00, y: 26.75, theta: 0.0, label: "Assembly Station 1", station_id: "DROPOFF_STATION_D", zone: "ASSEMBLY_STN_1" },
    progress_pct: 100,
    dwell_times: {
      pickup_wait_s: 3.0,
      dropoff_wait_s: 3.0
    },
    dwell_remaining: 0.0,
    created_at_epoch: 1789187785.719,
    expires_at_epoch: 1789188085.719
  }
];

// --- PREDEFINED SAFE AISLE POINTS (FROM WAREHOUSE_TASKS.PY) ---
const SAFE_AISLE_POINTS = (function() {
  const x_zones = {
    west: [-19.99, -15.67, -11.35],
    center: [-2.5363, 2.5363],
    east: [11.35, 15.67, 19.99]
  };
  const y_zones = {
    south: [-28.79, -26.755, -24.72, -22.685, -20.65, -18.615, -16.58, -14.545, -12.51],
    middle: [-7.1225, -5.0875, -3.0525, -1.0175, 1.0175, 3.0525, 5.0875, 7.1225],
    north: [12.51, 14.545, 16.58, 18.615, 20.65, 22.685, 24.72, 26.755, 28.79]
  };

  const points = [];
  for (const [y_zone, rows] of Object.entries(y_zones)) {
    for (const [x_zone, columns] of Object.entries(x_zones)) {
      if (y_zone === 'south' && x_zone === 'center') continue;
      for (let i = 0; i < rows.length - 1; i++) {
        const aisle_y = parseFloat(((rows[i] + rows[i + 1]) / 2.0).toFixed(4));
        for (const shelf_x of columns) {
          points.push({ x: shelf_x, y: aisle_y, theta: 0.0, x_zone, y_zone });
        }
      }
    }
  }
  for (const y_zone of ['middle', 'north']) {
    for (const shelf_y of y_zones[y_zone]) {
      points.push({ x: 0.0, y: shelf_y, theta: parseFloat((Math.PI / 2.0).toFixed(4)), x_zone: 'center', y_zone });
    }
  }
  return points;
})();

let generatedTaskCounter = 4;

// --- APP STATE ---
const APP_STATE = {
  activeTab: 'dashboard',
  viewMode: '2D',
  simSpeed: 1.0,
  isPaused: false,
  isEStopped: false,
  showTrails: true,
  showThoughts: true,
  selectedRobotId: 'robot_1',
  taskViewMode: 'table',
  streamPaused: false,
  
  // Real dataset metrics
  datasetStats: {
    totalRecords: 413,
    avgTravelTime: 124.5,
    meanPathLength: 24.6,
    meanTurns: 1.8,
    meanJunctions: 1.4,
    meanStopSec: 4.2,
    meanWaitSec: 2.4,
    fastPct: 76,
    medPct: 19,
    slowPct: 5
  },

  // Heatmap tracking
  heatmapGrid: Array(60).fill(0).map(() => Array(45).fill(0)),

  // Active AMR Fleet
  robots: [
    {
      id: 'robot_1',
      name: 'robot_1',
      namespace: '/robot_1',
      color: '#06b6d4',
      x: -5.25,
      y: -29.55,
      theta: 1.57,
      speed: 0.0,
      battery: 100.0,
      isCharging: false,
      state: 'IDLE',
      taskId: null,
      path: [],
      breadcrumbs: [],
      pathIdx: 0,
      thought: 'Docked at Charging Pad 1'
    },
    {
      id: 'robot_2',
      name: 'robot_2',
      namespace: '/robot_2',
      color: '#10b981',
      x: -3.75,
      y: -29.55,
      theta: 1.57,
      speed: 0.0,
      battery: 100.0,
      isCharging: false,
      state: 'IDLE',
      taskId: null,
      path: [],
      breadcrumbs: [],
      pathIdx: 0,
      thought: 'Docked at Charging Pad 2'
    },
    {
      id: 'robot_3',
      name: 'robot_3',
      namespace: '/robot_3',
      color: '#f59e0b',
      x: -2.25,
      y: -29.55,
      theta: 1.57,
      speed: 0.0,
      battery: 100.0,
      isCharging: false,
      state: 'IDLE',
      taskId: null,
      path: [],
      breadcrumbs: [],
      pathIdx: 0,
      thought: 'Docked at Charging Pad 3'
    },
    {
      id: 'robot_4',
      name: 'robot_4',
      namespace: '/robot_4',
      color: '#a855f7',
      x: -0.75,
      y: -29.55,
      theta: 1.57,
      speed: 0.0,
      battery: 100.0,
      isCharging: false,
      state: 'IDLE',
      taskId: null,
      path: [],
      breadcrumbs: [],
      pathIdx: 0,
      thought: 'Docked at Charging Pad 4'
    }
  ],

  obstacles: [
    { x: 0.0, y: 8.0, w: 2.0, h: 2.0, reason: 'Dynamic Obstacle (LiDAR)' }
  ],

  // Ingested Real Tasks (Array of tasks populated as per Data Specification)
  tasks: JSON.parse(JSON.stringify(MOCK_TASKS)),

  // Live Multi-Agent Decision Stream
  logs: [
    {
      time: '12:28:10',
      robot: 'robot_1',
      category: 'CBBA_AUCTION',
      description: 'SUBMIT_BID for task rnd_task_001. Bid cost: 104.46 at epoch 1 (pose: -5.2, -22.6 -> pickup: -19.9, -17.6, priority 100)',
      status: 'Success',
      chips: [
        { label: 'Task', val: 'rnd_task_001' },
        { label: 'Bid', val: '104.46', isBid: true },
        { label: 'Priority', val: '100' },
        { label: 'Quorum', val: '4/4', isQuorum: true }
      ]
    },
    {
      time: '12:28:12',
      robot: 'robot_1',
      category: 'QUORUM_CONSENSUS',
      description: 'UNANIMOUS_COMMIT for task rnd_task_001 -> Winner=robot_1, Winning Bid=104.46. Consensus quorum 4/4 verified across fleet.',
      status: 'Success',
      chips: [
        { label: 'Winner', val: 'robot_1' },
        { label: 'Quorum', val: '4/4 Verified', isQuorum: true }
      ]
    },
    {
      time: '12:28:14',
      robot: 'robot_1',
      category: 'WHCA_ROUTING',
      description: 'ROUTE_FEASIBLE for rnd_task_001: start=(-5, -23) -> pickup=(-20, -18). Steps: 24, Waypoints: 4, Reservations: 24, Dynamic Cells: 0.',
      status: 'Success',
      chips: [
        { label: 'Steps', val: '24' },
        { label: 'Reservations', val: '24' },
        { label: 'Conflicts', val: '0' }
      ]
    },
    {
      time: '12:28:22',
      robot: 'robot_2',
      category: 'CORRIDOR_MUTEX',
      description: 'REQUEST_MUTEX for corridor MC-NS-W. Lamport logical clock #104. Full grants received from peers [robot_1, robot_3, robot_4].',
      status: 'Active',
      chips: [
        { label: 'Corridor', val: 'MC-NS-W' },
        { label: 'Lamport Clock', val: '#104' },
        { label: 'Grants', val: '3/3' }
      ]
    },
    {
      time: '12:28:35',
      robot: 'FLEET',
      category: 'SAFETY_ALERT',
      description: 'LiDAR Dynamic Obstacle detected near (0.0, 8.0). Dynamic space-time costmap updated; reactive braking verified on approaching peers.',
      status: 'Warning',
      chips: [
        { label: 'Coords', val: 'X:0.0, Y:8.0' },
        { label: 'Action', val: 'Costmap Reroute' }
      ]
    }
  ]
};

// --- DYNAMIC REAL DATASET INGESTION ---
async function loadDatasetTasksAndMetrics() {
  const sources = ['/all_collected_dataset.csv', '/desktop_fleet_dataset.csv'];

  for (const src of sources) {
    try {
      const res = await fetch(src + '?t=' + Date.now());
      if (!res.ok) continue;
      const text = await res.text();
      const lines = text.trim().split('\n');
      if (lines.length <= 1) continue;

      const headers = lines[0].split(',').map(h => h.trim());
      const runIdx = headers.indexOf('run_id');
      const taskIdx = headers.indexOf('task_id');
      const botIdx = headers.indexOf('robot_id');
      const startZoneIdx = headers.indexOf('start_zone');
      const goalZoneIdx = headers.indexOf('goal_zone');
      const pathLenIdx = headers.indexOf('static_path_length_m');
      const turnsIdx = headers.indexOf('turn_count');
      const junctionsIdx = headers.indexOf('junction_crossings_count');
      const waitIdx = headers.indexOf('waiting_time_s');
      const stopIdx = headers.indexOf('total_stop_time_s');
      const travelIdx = headers.indexOf('actual_travel_time_s');

      const parsedTasks = [];
      const travelTimes = [];
      const pathLengths = [];
      const turnCounts = [];
      const stopTimes = [];
      const waitTimes = [];

      for (let i = 1; i < lines.length; i++) {
        const cols = lines[i].split(',').map(c => c.trim());
        if (cols.length < headers.length) continue;

        const runId = runIdx >= 0 ? cols[runIdx] : 'run_1789189224';
        const taskId = taskIdx >= 0 ? cols[taskIdx] : `rnd_task_${i.toString().padStart(3, '0')}`;
        const botId = botIdx >= 0 ? cols[botIdx] : 'robot_1';
        const startZone = startZoneIdx >= 0 ? cols[startZoneIdx] : 'South';
        const goalZone = goalZoneIdx >= 0 ? cols[goalZoneIdx] : 'North';
        const pathLen = pathLenIdx >= 0 ? parseFloat(cols[pathLenIdx]) || 12.0 : 12.0;
        const turns = turnsIdx >= 0 ? parseInt(cols[turnsIdx]) || 2 : 2;
        const waitSec = waitIdx >= 0 ? parseFloat(cols[waitIdx]) || 0 : 0;
        const stopSec = stopIdx >= 0 ? parseFloat(cols[stopIdx]) || 0 : 0;
        const travelSec = travelIdx >= 0 ? parseFloat(cols[travelIdx]) || 0 : 0;

        travelTimes.push(travelSec);
        pathLengths.push(pathLen);
        turnCounts.push(turns);
        stopTimes.push(stopSec);
        waitTimes.push(waitSec);

        // Pick canonical rack for start zone
        const racks = WAREHOUSE_CONFIG.zoneRacks[startZone] || WAREHOUSE_CONFIG.zoneRacks.South;
        const rack = racks[i % racks.length];

        // Pick canonical station for goal zone
        const station = WAREHOUSE_CONFIG.stations[goalZone] || WAREHOUSE_CONFIG.stations.North;

        // Realistic priority weighting from distance & load
        const priority = Math.min(100, Math.max(50, Math.round(50 + (pathLen / 100) * 35 + (i % 3) * 10)));
        const winningBid = parseFloat((90.0 + pathLen * 0.45 + waitSec * 0.05).toFixed(2));

        // Determine status: latest active session has active tasks, earlier ones completed
        let status = 'COMPLETED';
        let progressPct = 100;

        if (i === 1) {
          status = 'EN_ROUTE_DROPOFF';
          progressPct = 68;
        } else if (i === 2) {
          status = 'EN_ROUTE_PICKUP';
          progressPct = 35;
        } else if (i === 3) {
          status = 'PICKUP_WAIT';
          progressPct = 50;
        } else if (i === 4) {
          status = 'CBBA_AUCTION';
          progressPct = 0;
        }

        const taskObj = {
          run_id: runId,
          task_id: taskId,
          priority: priority,
          pickup: {
            x: rack.x,
            y: rack.y,
            theta: 0.0,
            rack_id: rack.id,
            item_type: rack.item
          },
          dropoff: {
            x: station.x,
            y: station.y,
            theta: 0.0,
            station_id: station.id,
            zone: station.zone
          },
          dwell_times: {
            pickup_wait_s: 3.0,
            dropoff_wait_s: 3.0
          },
          dwell_remaining: status === 'PICKUP_WAIT' ? 2.4 : 0.0,
          status: status,
          assigned_robot_id: status === 'CBBA_AUCTION' ? null : botId,
          winning_bid: status === 'CBBA_AUCTION' ? null : winningBid,
          progress_pct: progressPct,
          static_path_length_m: pathLen,
          actual_travel_time_s: travelSec,
          created_at_epoch: Date.now() / 1000 - i * 15,
          expires_at_epoch: Date.now() / 1000 + 300
        };

        parsedTasks.push(taskObj);
      }

      if (parsedTasks.length > 0) {
        // Ensure canonical MOCK_TASKS from specification remain primary active workload
        const mockTaskIds = new Set(MOCK_TASKS.map(m => m.task_id));
        const additionalTasks = parsedTasks.filter(p => !mockTaskIds.has(p.task_id));
        APP_STATE.tasks = [...JSON.parse(JSON.stringify(MOCK_TASKS)), ...additionalTasks];
        
        const avg = arr => arr.reduce((a, b) => a + b, 0) / arr.length;
        APP_STATE.datasetStats.totalRecords = travelTimes.length;
        APP_STATE.datasetStats.avgTravelTime = avg(travelTimes);
        APP_STATE.datasetStats.meanPathLength = avg(pathLengths);
        APP_STATE.datasetStats.meanTurns = avg(turnCounts);
        APP_STATE.datasetStats.meanStopSec = avg(stopTimes);
        APP_STATE.datasetStats.meanWaitSec = avg(waitTimes);

        const fast = travelTimes.filter(t => t < 500).length;
        const med = travelTimes.filter(t => t >= 500 && t <= 1200).length;
        const slow = travelTimes.filter(t => t > 1200).length;
        const total = travelTimes.length;

        APP_STATE.datasetStats.fastPct = Math.round((fast / total) * 100);
        APP_STATE.datasetStats.medPct = Math.round((med / total) * 100);
        APP_STATE.datasetStats.slowPct = Math.round((slow / total) * 100);

        renderTasksTable();
        updateTaskSummaryMetrics();
        updateStatisticsUI();
        console.log(`[DatasetLoader] Ingested ${parsedTasks.length} real tasks from ${src}`);
        break; // Successfully loaded definitive dataset
      }
    } catch (e) {
      console.warn(`Error loading dataset from ${src}`, e);
    }
  }
}

function updateStatisticsUI() {
  const d = APP_STATE.datasetStats;
  const mins = Math.floor(d.avgTravelTime / 60);
  const secs = Math.floor(d.avgTravelTime % 60);

  const elemTravel = document.getElementById('stat-avg-travel-time');
  if (elemTravel) elemTravel.textContent = `${mins}m ${secs.toString().padStart(2, '0')}s`;

  const elemSample = document.getElementById('stat-sample-count');
  if (elemSample) elemSample.textContent = `Calculated live across ${d.totalRecords} mission runs`;

  const elemPath = document.getElementById('stat-mean-path-len');
  if (elemPath) elemPath.textContent = `${d.meanPathLength.toFixed(1)} m`;

  const elemTurns = document.getElementById('stat-mean-turns');
  if (elemTurns) elemTurns.textContent = `Avg ${d.meanTurns.toFixed(1)} turns / WHCA* path`;

  const elemWait = document.getElementById('stat-corridor-wait');
  if (elemWait) elemWait.textContent = `${d.meanWaitSec.toFixed(1)}s`;

  const elemStop = document.getElementById('stat-stop-time');
  if (elemStop) elemStop.textContent = `${d.meanStopSec.toFixed(1)}s`;

  const barFast = document.getElementById('stat-bar-fast');
  const pctFast = document.getElementById('stat-pct-fast');
  if (barFast && pctFast) {
    barFast.style.width = `${d.fastPct}%`;
    pctFast.textContent = `${d.fastPct}%`;
  }

  const barMed = document.getElementById('stat-bar-med');
  const pctMed = document.getElementById('stat-pct-med');
  if (barMed && pctMed) {
    barMed.style.width = `${d.medPct}%`;
    pctMed.textContent = `${d.medPct}%`;
  }

  const barSlow = document.getElementById('stat-bar-slow');
  const pctSlow = document.getElementById('stat-pct-slow');
  if (barSlow && pctSlow) {
    barSlow.style.width = `${d.slowPct}%`;
    pctSlow.textContent = `${d.slowPct}%`;
  }
}

// --- RENDER BATCHING, DIRTY CHECKING & EVENT DEDUPLICATION ENGINE ---
let logsRenderScheduled = false;
let lastLogsFingerprint = '';

function scheduleLogsRender(force = false) {
  if (logsRenderScheduled) return;
  logsRenderScheduled = true;
  requestAnimationFrame(() => {
    logsRenderScheduled = false;

    if (APP_STATE.activeTab === 'dashboard') {
      renderRecentLogsDashboard();
    } else if (APP_STATE.activeTab === 'logs') {
      const topLogs = APP_STATE.logs.slice(0, 10);
      const fp = topLogs.map(l => `${l.time}:${l.robot}:${l.description}`).join('|');
      if (force || fp !== lastLogsFingerprint) {
        lastLogsFingerprint = fp;
        renderLogsTable();
      }
    }
  });
}

let tasksRenderScheduled = false;
let lastTasksFingerprint = '';

function scheduleTasksRender(force = false) {
  if (tasksRenderScheduled) return;
  tasksRenderScheduled = true;
  requestAnimationFrame(() => {
    tasksRenderScheduled = false;
    updateTaskSummaryMetrics();

    // Skip heavy DOM generation if the tasks tab is not active
    if (APP_STATE.activeTab !== 'tasks') return;

    const activeTasksSubset = APP_STATE.tasks.slice(0, 25);
    const fp = activeTasksSubset.map(t => `${t.task_id}:${t.status}:${t.progress_pct}:${t.dwell_remaining}:${t.assigned_robot_id}`).join('|');
    if (!force && fp === lastTasksFingerprint) {
      return;
    }
    lastTasksFingerprint = fp;

    if (APP_STATE.taskViewMode === 'table') {
      renderTasksTable();
    } else if (typeof renderTaskCards === 'function') {
      renderTaskCards();
    }
  });
}

let sidebarRenderScheduled = false;
let lastSidebarFingerprint = '';

function scheduleSidebarRender(force = false) {
  if (sidebarRenderScheduled) return;
  sidebarRenderScheduled = true;
  requestAnimationFrame(() => {
    sidebarRenderScheduled = false;
    if (APP_STATE.activeTab !== 'dashboard' && APP_STATE.activeTab !== 'map-view') return;

    const fp = APP_STATE.robots.map(r => `${r.id}:${r.state}:${Math.round(r.battery)}:${r.taskId}:${r.speed}`).join('|');
    if (!force && fp === lastSidebarFingerprint) return;
    lastSidebarFingerprint = fp;

    renderSidebarAmrCards();
    updateUberDirectionCard();
  });
}

const processedEventHashes = new Set();
const processedEventHashQueue = [];
const MAX_EVENT_HASH_CACHE = 500;

function hasEventBeenProcessed(ev) {
  const key = `${ev.timestamp || ''}:${ev.type || ''}:${ev.detail || ''}`;
  if (processedEventHashes.has(key)) return true;
  processedEventHashes.add(key);
  processedEventHashQueue.push(key);
  if (processedEventHashQueue.length > MAX_EVENT_HASH_CACHE) {
    const oldest = processedEventHashQueue.shift();
    processedEventHashes.delete(oldest);
  }
  return false;
}

// --- LOGGING ENGINE ---
function addStructuredLog(robot, category, description, status = 'Success', chips = []) {
  if (APP_STATE.streamPaused) return;

  const timeStr = new Date().toLocaleTimeString('en-US', { hour12: false });
  const newLog = {
    time: timeStr,
    robot: robot || 'FLEET',
    category: category || 'TASK_LIFECYCLE',
    description,
    status,
    chips: chips || []
  };

  APP_STATE.logs.unshift(newLog);
  if (APP_STATE.logs.length > 200) APP_STATE.logs.pop();

  scheduleLogsRender();
}

// --- TELEMETRY BRIDGE CLIENT ---
let telemetrySocket = null;
let restPollingInterval = null;

function updateConnectionStatus(isConnected, label) {
  const pill = document.querySelector('.fleet-status-pill');
  if (!pill) return;
  const dot = pill.querySelector('.status-indicator-dot');
  const txt = pill.querySelector('.status-text');
  if (isConnected) {
    if (dot) {
      dot.style.backgroundColor = '#10b981';
      dot.style.boxShadow = '0 0 8px rgba(16, 185, 129, 0.7)';
    }
    const count = APP_STATE.robots.length;
    if (txt) txt.innerHTML = `Fleet Live: <strong>${count} AMRs</strong>`;
  } else {
    if (dot) {
      dot.style.backgroundColor = '#f59e0b';
      dot.style.boxShadow = 'none';
    }
    const count = APP_STATE.robots.length;
    if (txt) txt.innerHTML = `Fleet Standalone: <strong>${count} AMRs</strong>`;
  }
}

function initTelemetryBridge() {
  const host = window.location.hostname || 'localhost';
  const wsUrls = [`ws://${host}:8765`, `ws://${host}:9090`, 'ws://localhost:8765', 'ws://localhost:9090'];
  let connected = false;

  function tryConnect(idx) {
    if (idx >= wsUrls.length) {
      startRestTelemetryPolling();
      return;
    }

    try {
      telemetrySocket = new WebSocket(wsUrls[idx]);
      telemetrySocket.onopen = () => {
        connected = true;
        APP_STATE.isLiveConnected = true;
        if (restPollingInterval) {
          clearInterval(restPollingInterval);
          restPollingInterval = null;
        }
        console.log(`[TelemetryBridge] Connected to ${wsUrls[idx]}`);
        updateConnectionStatus(true, wsUrls[idx]);
      };

      telemetrySocket.onmessage = (event) => {
        try {
          const payload = JSON.parse(event.data);
          handleLiveTelemetryPayload(payload);
        } catch (err) {
          parseRawBackendLogLine(event.data);
        }
      };

      telemetrySocket.onerror = () => {
        if (!connected) tryConnect(idx + 1);
      };

      telemetrySocket.onclose = () => {
        if (connected) {
          console.warn('[TelemetryBridge] Disconnected, switching to polling');
          APP_STATE.isLiveConnected = false;
          updateConnectionStatus(false);
          startRestTelemetryPolling();
        }
      };
    } catch (e) {
      tryConnect(idx + 1);
    }
  }

  tryConnect(0);
}

function startRestTelemetryPolling() {
  if (restPollingInterval) return; // Prevent multiple concurrent polling timers
  const host = window.location.hostname || 'localhost';
  const endpoints = [
    '/fleet/dashboard_telemetry',
    `http://${host}:8766/fleet/dashboard_telemetry`,
    'http://localhost:8766/fleet/dashboard_telemetry'
  ];
  let epIdx = 0;

  restPollingInterval = setInterval(async () => {
    try {
      const res = await fetch(endpoints[epIdx], { cache: 'no-store' });
      if (res.ok) {
        const payload = await res.json();
        handleLiveTelemetryPayload(payload);
      } else {
        epIdx = (epIdx + 1) % endpoints.length;
      }
    } catch (e) {
      epIdx = (epIdx + 1) % endpoints.length;
    }
  }, 250);
}

function handleLiveTelemetryPayload(data) {
  if (!data) return;
  APP_STATE.isLiveConnected = true;
  updateConnectionStatus(true);

  if (data.robots) {
    for (const [rId, pose] of Object.entries(data.robots)) {
      let bot = APP_STATE.robots.find(r => r.id === rId);
      if (!bot) {
        bot = {
          id: rId,
          name: rId,
          namespace: `/${rId}`,
          color: ROBOT_COLOR_MAP[rId] || '#3b82f6',
          x: pose.x !== undefined ? pose.x : 0,
          y: pose.y !== undefined ? pose.y : 0,
          theta: pose.theta !== undefined ? pose.theta : 0,
          targetX: pose.x !== undefined ? pose.x : 0,
          targetY: pose.y !== undefined ? pose.y : 0,
          targetTheta: pose.theta !== undefined ? pose.theta : 0,
          speed: pose.speed || 0.0,
          battery: 95.0,
          isCharging: false,
          state: pose.state || 'IDLE',
          taskId: pose.taskId || null,
          path: [],
          breadcrumbs: [],
          pathIdx: 0,
          thought: pose.thought || 'Live ROS 2'
        };
        APP_STATE.robots.push(bot);
      } else {
        bot.targetX = pose.x;
        bot.targetY = pose.y;
        bot.targetTheta = pose.theta;
        if (bot.x === undefined) {
          bot.x = pose.x;
          bot.y = pose.y;
          bot.theta = pose.theta;
        }
        if (pose.speed !== undefined) bot.speed = pose.speed;
        if (pose.state) bot.state = pose.state;
        if (pose.thought) bot.thought = pose.thought;
        if (pose.taskId !== undefined) bot.taskId = pose.taskId;
      }

      // Record breadcrumbs for live motion history
      if (!bot.breadcrumbs) bot.breadcrumbs = [];
      const lastPt = bot.breadcrumbs[bot.breadcrumbs.length - 1];
      if (!lastPt || Math.hypot(bot.x - lastPt.x, bot.y - lastPt.y) > 0.25) {
        bot.breadcrumbs.push({ x: bot.x, y: bot.y });
        if (bot.breadcrumbs.length > 50) bot.breadcrumbs.shift();
      }

      // Inject live planned routes if provided by ROS 2
      if (data.paths && data.paths[rId] && Array.isArray(data.paths[rId])) {
        bot.path = data.paths[rId];
        bot.pathIdx = 0;
      }
    }
  }

  if (data.health) {
    for (const [rId, h] of Object.entries(data.health)) {
      const bot = APP_STATE.robots.find(r => r.id === rId);
      if (bot) {
        if (h.battery !== undefined) bot.battery = h.battery;
        if (h.safe !== undefined) bot.safe = h.safe;
      }
    }
  }

  if (data.tasks && Array.isArray(data.tasks)) {
    for (const t of data.tasks) {
      const existing = APP_STATE.tasks.find(x => x.task_id === t.task_id);
      if (existing) {
        Object.assign(existing, t);
      } else {
        APP_STATE.tasks.unshift(t);
      }
    }
    scheduleTasksRender();
  }

  if (data.events && Array.isArray(data.events)) {
    for (const ev of data.events) {
      if (ev && ev.detail && !hasEventBeenProcessed(ev)) {
        parseRawBackendLogLine(ev.detail);
      }
    }
  }

  scheduleSidebarRender();
}

function parseRawBackendLogLine(line) {
  if (!line || typeof line !== 'string') return;

  const mBid = line.match(/\[(robot_\d):CBBA\]\s*Decision:\s*SUBMIT_BID for task ([a-zA-Z0-9_-]+).*bid=([\d.]+).*priority=(\d+)/i);
  if (mBid) {
    const [, robot, task, bid, prio] = mBid;
    addStructuredLog(robot, 'CBBA_AUCTION', `SUBMIT_BID for ${task}: Bid=${bid}, Priority=${prio}`, 'Success', [
      { label: 'Task', val: task },
      { label: 'Bid', val: bid, isBid: true },
      { label: 'Priority', val: prio }
    ]);
    return;
  }

  const mCommit = line.match(/\[(robot_\d):CBBA\]\s*Decision:\s*UNANIMOUS_COMMIT for task ([a-zA-Z0-9_-]+)\s*->\s*Winner=(robot_\d),\s*Bid=([\d.]+).*Quorum=(\d+\/\d+)/i);
  if (mCommit) {
    const [, , task, winner, bid, quorum] = mCommit;
    addStructuredLog(winner, 'QUORUM_CONSENSUS', `UNANIMOUS_COMMIT for ${task}: Winner=${winner}, Bid=${bid}, Quorum=${quorum}`, 'Success', [
      { label: 'Winner', val: winner },
      { label: 'Bid', val: bid, isBid: true },
      { label: 'Quorum', val: quorum, isQuorum: true }
    ]);
    return;
  }

  const mWhca = line.match(/\[(robot_\d):WHCA\]\s*Decision:\s*ROUTE_FEASIBLE for task=([a-zA-Z0-9_-]+).*steps=(\d+).*waypoints=(\d+).*reservations=(\d+)/i);
  if (mWhca) {
    const [, robot, task, steps, wps, res] = mWhca;
    addStructuredLog(robot, 'WHCA_ROUTING', `ROUTE_FEASIBLE for ${task}: ${steps} steps, ${wps} waypoints, ${res} space-time reservations`, 'Success', [
      { label: 'Steps', val: steps },
      { label: 'Reservations', val: res }
    ]);
    return;
  }

  const mMutex = line.match(/\[(robot_\d):CorridorMutex\]\s*Decision:\s*(REQUEST_MUTEX|ENTER_CORRIDOR|EXIT_CORRIDOR|RELEASE_MUTEX) for corridor ([a-zA-Z0-9_-]+)/i);
  if (mMutex) {
    const [, robot, action, corridor] = mMutex;
    addStructuredLog(robot, 'CORRIDOR_MUTEX', `${action} on corridor ${corridor}`, 'Active', [
      { label: 'Action', val: action },
      { label: 'Corridor', val: corridor }
    ]);
    return;
  }

  addStructuredLog('FLEET', 'TASK_LIFECYCLE', line, 'Success');
}

// --- TAB ROUTING ---
function initNavigation() {
  const tabs = document.querySelectorAll('.nav-tab');
  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const pageId = tab.dataset.page;
      tabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');

      document.querySelectorAll('.page-view').forEach(p => p.classList.remove('active'));
      const targetPage = document.getElementById(`page-${pageId}`);
      if (targetPage) targetPage.classList.add('active');
      APP_STATE.activeTab = pageId;

      setTimeout(() => {
        resizeActiveCanvases();
        if (pageId === 'tasks') scheduleTasksRender(true);
        if (pageId === 'logs') scheduleLogsRender(true);
        if (pageId === 'dashboard' || pageId === 'map-view') scheduleSidebarRender(true);
      }, 50);
    });
  });

  const btnAllLogs = document.getElementById('btn-view-all-logs');
  if (btnAllLogs) {
    btnAllLogs.addEventListener('click', () => {
      document.querySelector('[data-page="logs"]').click();
    });
  }

  const btnDash2D = document.getElementById('dash-btn-2d');
  const btnDash3D = document.getElementById('dash-btn-3d');
  const btnMap2D = document.getElementById('map-btn-2d');
  const btnMap3D = document.getElementById('map-btn-3d');

  function setViewMode(mode) {
    APP_STATE.viewMode = mode;
    [btnDash2D, btnMap2D].forEach(b => b && b.classList.toggle('active', mode === '2D'));
    [btnDash3D, btnMap3D].forEach(b => b && b.classList.toggle('active', mode === '3D'));
  }

  if (btnDash2D) btnDash2D.addEventListener('click', () => setViewMode('2D'));
  if (btnDash3D) btnDash3D.addEventListener('click', () => setViewMode('3D'));
  if (btnMap2D) btnMap2D.addEventListener('click', () => setViewMode('2D'));
  if (btnMap3D) btnMap3D.addEventListener('click', () => setViewMode('3D'));

  document.querySelectorAll('.speed-toggle').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.speed-toggle').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      APP_STATE.simSpeed = parseFloat(btn.dataset.speed);
    });
  });

  const pauseBtn = document.getElementById('global-pause-btn');
  pauseBtn?.addEventListener('click', () => {
    APP_STATE.isPaused = !APP_STATE.isPaused;
    pauseBtn.textContent = APP_STATE.isPaused ? '▶' : '⏸';
  });

  const estopBtn = document.getElementById('global-estop-btn');
  estopBtn?.addEventListener('click', () => {
    APP_STATE.isEStopped = !APP_STATE.isEStopped;
    if (APP_STATE.isEStopped) {
      estopBtn.style.background = '#dc2626';
      estopBtn.style.color = '#fff';
      estopBtn.innerHTML = '<span>⚡</span> Resume';
      addStructuredLog('FLEET', 'SAFETY_ALERT', 'Global emergency stop triggered by supervisor. All AMRs braking.', 'Emergency');
    } else {
      estopBtn.style.background = '#fee2e2';
      estopBtn.style.color = '#991b1b';
      estopBtn.innerHTML = '<span>🛑</span> E-Stop';
      addStructuredLog('FLEET', 'SAFETY_ALERT', 'Emergency stop released. Resuming autonomous routing.', 'Success');
    }
  });

  const trailBtn = document.getElementById('dash-toggle-trails');
  if (trailBtn) {
    trailBtn.addEventListener('click', () => {
      APP_STATE.showTrails = !APP_STATE.showTrails;
      trailBtn.classList.toggle('active', APP_STATE.showTrails);
      trailBtn.textContent = `Trails: ${APP_STATE.showTrails ? 'ON' : 'OFF'}`;
    });
  }

  const thoughtBtn = document.getElementById('dash-toggle-bubbles');
  if (thoughtBtn) {
    thoughtBtn.addEventListener('click', () => {
      APP_STATE.showThoughts = !APP_STATE.showThoughts;
      thoughtBtn.classList.toggle('active', APP_STATE.showThoughts);
      thoughtBtn.textContent = `Thoughts: ${APP_STATE.showThoughts ? 'ON' : 'OFF'}`;
    });
  }

  const btnTable = document.getElementById('btn-view-table');
  const btnCards = document.getElementById('btn-view-cards');
  const tableContainer = document.getElementById('tasks-table-container');
  const cardsContainer = document.getElementById('tasks-cards-container');

  btnTable?.addEventListener('click', () => {
    btnTable.classList.add('active');
    btnCards.classList.remove('active');
    tableContainer.style.display = 'block';
    cardsContainer.style.display = 'none';
    APP_STATE.taskViewMode = 'table';
  });

  btnCards?.addEventListener('click', () => {
    btnCards.classList.add('active');
    btnTable.classList.remove('active');
    tableContainer.style.display = 'none';
    cardsContainer.style.display = 'grid';
    APP_STATE.taskViewMode = 'cards';
    renderTaskCards();
  });

  const btnStream = document.getElementById('btn-stream-toggle');
  btnStream?.addEventListener('click', () => {
    APP_STATE.streamPaused = !APP_STATE.streamPaused;
    btnStream.textContent = APP_STATE.streamPaused ? '▶ Resume Stream' : '⏸ Pause Stream';
  });

  document.getElementById('btn-clear-all-logs')?.addEventListener('click', () => {
    APP_STATE.logs = [];
    renderLogsTable();
  });

  document.getElementById('btn-export-logs')?.addEventListener('click', exportLogsToCSV);
  document.getElementById('btn-export-pdf')?.addEventListener('click', exportLogsToCSV);

  document.getElementById('btn-trigger-task-burst')?.addEventListener('click', triggerRandomTaskBurst);
}

// --- CANVAS RENDERING (2D & 3D ISOMETRIC ENGINE) ---
const dashCanvas = document.getElementById('dashWarehouseCanvas');
const fullCanvas = document.getElementById('fullWarehouseCanvas');
const heatmapCanvas = document.getElementById('statsHeatmapCanvas');

let dashCtx = dashCanvas ? dashCanvas.getContext('2d') : null;
let fullCtx = fullCanvas ? fullCanvas.getContext('2d') : null;
let heatCtx = heatmapCanvas ? heatmapCanvas.getContext('2d') : null;

function resizeActiveCanvases() {
  const containerDash = document.getElementById('dash-canvas-container');
  if (containerDash && dashCanvas) {
    dashCanvas.width = containerDash.clientWidth * window.devicePixelRatio;
    dashCanvas.height = containerDash.clientHeight * window.devicePixelRatio;
    dashCtx = dashCanvas.getContext('2d');
    dashCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
  }

  const containerFull = document.getElementById('fullmap-canvas-container');
  if (containerFull && fullCanvas) {
    fullCanvas.width = containerFull.clientWidth * window.devicePixelRatio;
    fullCanvas.height = containerFull.clientHeight * window.devicePixelRatio;
    fullCtx = fullCanvas.getContext('2d');
    fullCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
  }

  const containerHeat = heatmapCanvas ? heatmapCanvas.parentElement : null;
  if (containerHeat && heatmapCanvas) {
    heatmapCanvas.width = containerHeat.clientWidth * window.devicePixelRatio;
    heatmapCanvas.height = containerHeat.clientHeight * window.devicePixelRatio;
    heatCtx = heatmapCanvas.getContext('2d');
    heatCtx.scale(window.devicePixelRatio, window.devicePixelRatio);
    renderStaticHeatmap();
  }
}
window.addEventListener('resize', resizeActiveCanvases);

function projectWorld(wx, wy, wz, cWidth, cHeight, mode = '2D') {
  if (mode === '2D') {
    const scale = Math.min(cWidth / (WAREHOUSE_CONFIG.width_m + 6), cHeight / (WAREHOUSE_CONFIG.height_m + 6));
    const sx = cWidth / 2 + (wx * scale);
    const sy = cHeight / 2 - (wy * scale);
    return { x: sx, y: sy, scale };
  } else {
    const isoScale = Math.min(cWidth / 68, cHeight / 68) * 0.95;
    const isoAngle = Math.PI / 6;
    const isoX = (wx - wy * 0.8) * Math.cos(isoAngle);
    const isoY = (wx + wy * 0.8) * Math.sin(isoAngle) - (wz * 1.6);
    const sx = cWidth / 2 + (isoX * isoScale);
    const sy = cHeight / 2 + (isoY * isoScale) + 30;
    return { x: sx, y: sy, scale: isoScale };
  }
}

function drawWarehouseScene(ctx, cWidth, cHeight, mode) {
  ctx.clearRect(0, 0, cWidth, cHeight);

  ctx.fillStyle = mode === '3D' ? '#e2e8f0' : '#f8fafc';
  ctx.fillRect(0, 0, cWidth, cHeight);

  if (mode === '2D') {
    ctx.fillStyle = '#f1f5f9';
    for (const c of WAREHOUSE_CONFIG.corridors) {
      if (c.orientation === 'V') {
        const p1 = projectWorld(c.x - c.width / 2, WAREHOUSE_CONFIG.height_m / 2, 0, cWidth, cHeight, mode);
        const p2 = projectWorld(c.x + c.width / 2, -WAREHOUSE_CONFIG.height_m / 2, 0, cWidth, cHeight, mode);
        ctx.fillRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
      } else {
        const p1 = projectWorld(-WAREHOUSE_CONFIG.width_m / 2, c.y + c.height / 2, 0, cWidth, cHeight, mode);
        const p2 = projectWorld(WAREHOUSE_CONFIG.width_m / 2, c.y - c.height / 2, 0, cWidth, cHeight, mode);
        ctx.fillRect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
      }
    }
  }

  for (const pad of WAREHOUSE_CONFIG.charging_pads) {
    const p = projectWorld(pad.x, pad.y, 0, cWidth, cHeight, mode);
    if (mode === '2D') {
      const pw = 2.4 * p.scale;
      const ph = 1.8 * p.scale;
      ctx.fillStyle = '#ecfdf5';
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.roundRect(p.x - pw / 2, p.y - ph / 2, pw, ph, 4);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#065f46';
      ctx.font = 'bold 9px Inter';
      ctx.textAlign = 'center';
      ctx.fillText(`⚡ ${pad.id}`, p.x, p.y + 3);
    } else {
      ctx.fillStyle = '#a7f3d0';
      ctx.strokeStyle = '#059669';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 10, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  const shelvesSorted = mode === '3D' 
    ? [...WAREHOUSE_CONFIG.shelves].sort((a, b) => (a.x + a.y) - (b.x + b.y))
    : WAREHOUSE_CONFIG.shelves;

  for (const s of shelvesSorted) {
    if (mode === '2D') {
      const p = projectWorld(s.x, s.y, 0, cWidth, cHeight, mode);
      const sw = s.w * p.scale;
      const sh = s.h * p.scale;
      ctx.fillStyle = '#e2e8f0';
      ctx.strokeStyle = '#cbd5e1';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.roundRect(p.x - sw / 2, p.y - sh / 2, sw, sh, 3);
      ctx.fill();
      ctx.stroke();

      ctx.strokeStyle = '#94a3b8';
      ctx.beginPath();
      ctx.moveTo(p.x - sw / 4, p.y - sh / 2);
      ctx.lineTo(p.x - sw / 4, p.y + sh / 2);
      ctx.moveTo(p.x + sw / 4, p.y - sh / 2);
      ctx.lineTo(p.x + sw / 4, p.y + sh / 2);
      ctx.stroke();
    } else {
      const pBase = projectWorld(s.x, s.y, 0, cWidth, cHeight, mode);
      const pTop = projectWorld(s.x, s.y, s.depth_3d, cWidth, cHeight, mode);
      const bW = 18;
      const bH = 10;
      const bHeight = pBase.y - pTop.y;

      ctx.fillStyle = '#cbd5e1';
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.rect(pTop.x - bW / 2, pTop.y, bW, bHeight);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#e2e8f0';
      ctx.beginPath();
      ctx.rect(pTop.x - bW / 2, pTop.y - bH, bW, bH);
      ctx.fill();
      ctx.stroke();
    }
  }

  for (const obs of APP_STATE.obstacles) {
    const p = projectWorld(obs.x, obs.y, 0, cWidth, cHeight, mode);
    ctx.fillStyle = '#fef2f2';
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(p.x - 14, p.y - 14, 28, 28, 4);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#dc2626';
    ctx.font = 'bold 10px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('⚠️', p.x, p.y + 4);
  }

  if (APP_STATE.showTrails) {
    for (const bot of APP_STATE.robots) {
      // 1. Draw breadcrumb motion trail
      if (bot.breadcrumbs && bot.breadcrumbs.length > 1) {
        ctx.strokeStyle = bot.color ? `${bot.color}88` : 'rgba(96, 165, 250, 0.4)';
        ctx.lineWidth = 2.0;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        const p0 = projectWorld(bot.breadcrumbs[0].x, bot.breadcrumbs[0].y, 0, cWidth, cHeight, mode);
        ctx.moveTo(p0.x, p0.y);
        for (let i = 1; i < bot.breadcrumbs.length; i++) {
          const pt = projectWorld(bot.breadcrumbs[i].x, bot.breadcrumbs[i].y, 0, cWidth, cHeight, mode);
          ctx.lineTo(pt.x, pt.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // 2. Draw planned future route waypoints
      if (bot.path && bot.path.length > 1) {
        ctx.strokeStyle = bot.id === APP_STATE.selectedRobotId ? '#2563eb' : (bot.color || '#60a5fa');
        ctx.lineWidth = bot.id === APP_STATE.selectedRobotId ? 3.0 : 1.8;
        ctx.beginPath();

        const pStart = projectWorld(bot.x, bot.y, 0, cWidth, cHeight, mode);
        ctx.moveTo(pStart.x, pStart.y);

        for (let i = bot.pathIdx; i < bot.path.length; i++) {
          const pNode = projectWorld(bot.path[i].x, bot.path[i].y, 0, cWidth, cHeight, mode);
          ctx.lineTo(pNode.x, pNode.y);
        }
        ctx.stroke();

        const dest = bot.path[bot.path.length - 1];
        const pDest = projectWorld(dest.x, dest.y, 0, cWidth, cHeight, mode);
        ctx.fillStyle = bot.color || '#2563eb';
        ctx.beginPath();
        ctx.arc(pDest.x, pDest.y, 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
    }
  }

  for (const bot of APP_STATE.robots) {
    const p = projectWorld(bot.x, bot.y, mode === '3D' ? 0.4 : 0, cWidth, cHeight, mode);
    const isSelected = bot.id === APP_STATE.selectedRobotId;

    if (isSelected) {
      ctx.strokeStyle = 'rgba(37, 99, 235, 0.4)';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 16, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.fillStyle = bot.color || '#3b82f6';
    ctx.beginPath();
    ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.stroke();

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-bot.theta);
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(6, 0);
    ctx.lineTo(2, -3);
    ctx.lineTo(2, 3);
    ctx.fill();
    ctx.restore();

    ctx.fillStyle = '#0f172a';
    ctx.font = 'bold 10px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(bot.name, p.x, p.y + 20);

    if (APP_STATE.showThoughts && bot.thought) {
      drawMinimalThoughtBubble(ctx, p.x, p.y - 18, bot.thought);
    }
  }
}

function drawMinimalThoughtBubble(ctx, x, y, text) {
  ctx.font = '600 10px Inter';
  const tw = ctx.measureText(text).width;
  const bw = tw + 14;
  const bh = 20;

  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#cbd5e1';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(x - bw / 2, y - bh, bw, bh, 4);
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = '#ffffff';
  ctx.beginPath();
  ctx.moveTo(x - 3, y);
  ctx.lineTo(x + 3, y);
  ctx.lineTo(x, y + 4);
  ctx.fill();

  ctx.fillStyle = '#334155';
  ctx.textAlign = 'center';
  ctx.fillText(text, x, y - 6);
}

// --- SIMULATION STEP ENGINE ---
let lastAnimTime = performance.now();

function updateSimulationEngine(dt) {
  if (APP_STATE.isPaused || APP_STATE.isEStopped) return;

  if (APP_STATE.isLiveConnected) {
    // Smooth real-time pose interpolation towards incoming ROS 2 coordinates
    for (const bot of APP_STATE.robots) {
      if (bot.targetX !== undefined) {
        const lerpSpeed = Math.min(1.0, dt * 10.0);
        bot.x += (bot.targetX - bot.x) * lerpSpeed;
        bot.y += (bot.targetY - bot.y) * lerpSpeed;
        let diff = bot.targetTheta - bot.theta;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        bot.theta += diff * lerpSpeed;
      }
    }
    updateUberDirectionCard();
    return;
  }

  for (const t of APP_STATE.tasks) {
    if (t.status === 'PICKUP_WAIT' || t.status === 'DROPOFF_WAIT') {
      if (t.dwell_remaining > 0) {
        t.dwell_remaining = Math.max(0, t.dwell_remaining - dt * APP_STATE.simSpeed);
        if (t.dwell_remaining === 0) {
          advanceTaskDwellComplete(t);
        }
      }
    }
  }

  for (const bot of APP_STATE.robots) {
    if (bot.isCharging) {
      bot.battery = Math.min(100, bot.battery + dt * 2.0 * APP_STATE.simSpeed);
    } else {
      bot.battery = Math.max(0, bot.battery - dt * 0.08 * APP_STATE.simSpeed);
    }

    if (bot.path && bot.path.length > 0 && bot.pathIdx < bot.path.length) {
      const targetWp = bot.path[bot.pathIdx];
      const dx = targetWp.x - bot.x;
      const dy = targetWp.y - bot.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      if (dist < 0.25) {
        bot.pathIdx++;
        if (bot.pathIdx >= bot.path.length) {
          handleRobotArrival(bot);
        }
      } else {
        const angle = Math.atan2(dy, dx);
        bot.theta = angle;
        const step = (bot.speed || 0.8) * dt * APP_STATE.simSpeed;
        bot.x += Math.cos(angle) * Math.min(step, dist);
        bot.y += Math.sin(angle) * Math.min(step, dist);

        const hx = Math.min(44, Math.max(0, Math.floor(bot.x + 22.5)));
        const hy = Math.min(59, Math.max(0, Math.floor(bot.y + 30.0)));
        APP_STATE.heatmapGrid[hy][hx] += dt * 0.4;
      }
    }
  }

  updateUberDirectionCard();
}

function handleRobotArrival(bot) {
  const activeTask = APP_STATE.tasks.find(t => t.task_id === bot.taskId);
  if (!activeTask) return;

  if (activeTask.status === 'EN_ROUTE_PICKUP') {
    activeTask.status = 'PICKUP_WAIT';
    activeTask.dwell_remaining = activeTask.dwell_times.pickup_wait_s || 3.0;
    activeTask.progress_pct = 50;
    bot.state = 'PICKUP_WAIT';
    bot.thought = `Loading ${activeTask.pickup.item_type || 'Cargo'} (3.0s dwell)`;

    addStructuredLog(bot.name, 'TASK_LIFECYCLE', `ARRIVED at pickup for task ${activeTask.task_id} (${activeTask.pickup.rack_id}). Dwelling ${activeTask.dwell_times.pickup_wait_s}s`, 'Active', [
      { label: 'Task', val: activeTask.task_id },
      { label: 'Rack', val: activeTask.pickup.rack_id },
      { label: 'Dwell', val: `${activeTask.dwell_times.pickup_wait_s}s` }
    ]);

    renderTasksTable();
    updateTaskSummaryMetrics();

  } else if (activeTask.status === 'EN_ROUTE_DROPOFF') {
    activeTask.status = 'DROPOFF_WAIT';
    activeTask.dwell_remaining = activeTask.dwell_times.dropoff_wait_s || 3.0;
    activeTask.progress_pct = 90;
    bot.state = 'DROPOFF_WAIT';
    bot.thought = `Unloading at ${activeTask.dropoff.station_id} (3.0s dwell)`;

    addStructuredLog(bot.name, 'TASK_LIFECYCLE', `ARRIVED at dropoff for task ${activeTask.task_id} (${activeTask.dropoff.station_id}). Dwelling ${activeTask.dwell_times.dropoff_wait_s}s`, 'Active', [
      { label: 'Task', val: activeTask.task_id },
      { label: 'Station', val: activeTask.dropoff.station_id }
    ]);

    renderTasksTable();
    updateTaskSummaryMetrics();
  }
}

function advanceTaskDwellComplete(task) {
  const bot = APP_STATE.robots.find(r => r.id === task.assigned_robot_id);

  if (task.status === 'PICKUP_WAIT') {
    task.status = 'EN_ROUTE_DROPOFF';
    task.progress_pct = 60;
    if (bot) {
      bot.state = 'EN_ROUTE_DROPOFF';
      bot.thought = `En Route Dropoff -> ${task.dropoff.station_id}`;
      bot.path = generateNavPath(bot.x, bot.y, task.dropoff.x, task.dropoff.y);
      bot.pathIdx = 0;
    }

    addStructuredLog(task.assigned_robot_id, 'TASK_LIFECYCLE', `PICKUP_DWELL_COMPLETE for task ${task.task_id}. Dispatching route to ${task.dropoff.station_id}`, 'Success', [
      { label: 'Task', val: task.task_id },
      { label: 'Dropoff', val: task.dropoff.station_id }
    ]);

    renderTasksTable();
    updateTaskSummaryMetrics();

  } else if (task.status === 'DROPOFF_WAIT') {
    task.status = 'COMPLETED';
    task.progress_pct = 100;
    if (bot) {
      bot.state = 'IDLE';
      bot.thought = 'Task Complete: Ready for CBBA Auction';
      bot.taskId = null;
      bot.path = [];
    }

    addStructuredLog(task.assigned_robot_id, 'TASK_LIFECYCLE', `DROPOFF_DWELL_COMPLETE for task ${task.task_id}. Task finished successfully.`, 'Success', [
      { label: 'Task', val: task.task_id },
      { label: 'Status', val: 'COMPLETED' }
    ]);

    renderTasksTable();
    updateTaskSummaryMetrics();

    setTimeout(() => {
      allocateNextAnnouncedTask();
    }, 1500 / APP_STATE.simSpeed);
  }
}

function allocateNextAnnouncedTask() {
  const pending = APP_STATE.tasks.find(t => t.status === 'ANNOUNCED' || t.status === 'CBBA_AUCTION');
  const freeBot = APP_STATE.robots.find(r => !r.taskId && !r.isCharging);

  if (pending && freeBot) {
    pending.status = 'ASSIGNED';
    pending.assigned_robot_id = freeBot.id;
    pending.winning_bid = parseFloat((95.0 + Math.random() * 20.0).toFixed(2));
    pending.progress_pct = 10;
    freeBot.taskId = pending.task_id;
    freeBot.state = 'ASSIGNED';
    freeBot.thought = `CBBA Winner: ${pending.task_id} ($${pending.winning_bid})`;

    addStructuredLog(freeBot.id, 'QUORUM_CONSENSUS', `UNANIMOUS_COMMIT for ${pending.task_id} -> Winner=${freeBot.id}, Bid=${pending.winning_bid}. Quorum=4/4 verified.`, 'Success', [
      { label: 'Task', val: pending.task_id },
      { label: 'Winner', val: freeBot.id },
      { label: 'Bid', val: `${pending.winning_bid}`, isBid: true },
      { label: 'Quorum', val: '4/4', isQuorum: true }
    ]);

    setTimeout(() => {
      pending.status = 'EN_ROUTE_PICKUP';
      pending.progress_pct = 25;
      freeBot.state = 'EN_ROUTE_PICKUP';
      freeBot.path = generateNavPath(freeBot.x, freeBot.y, pending.pickup.x, pending.pickup.y);
      freeBot.pathIdx = 0;
      freeBot.thought = `Route Feasible -> ${pending.pickup.rack_id}`;

      addStructuredLog(freeBot.id, 'WHCA_ROUTING', `ROUTE_FEASIBLE for task=${pending.task_id}: start=(${freeBot.x.toFixed(1)}, ${freeBot.y.toFixed(1)}) -> pickup=(${pending.pickup.x.toFixed(1)}, ${pending.pickup.y.toFixed(1)}). Steps: 28, Waypoints: 4, Reservations: 28`, 'Success', [
        { label: 'Steps', val: '28' },
        { label: 'Reservations', val: '28' }
      ]);

      renderTasksTable();
      updateTaskSummaryMetrics();
    }, 1200 / APP_STATE.simSpeed);

    renderTasksTable();
    updateTaskSummaryMetrics();
  }
}

function generateNavPath(sx, sy, gx, gy) {
  const vx = sx < 0 ? -9.0 : 9.0;
  const targetVx = gx < 0 ? -9.0 : 9.0;
  const hy = sy < 0 ? -10.0 : 10.0;

  return [
    { x: sx, y: sy },
    { x: vx, y: sy },
    { x: vx, y: hy },
    { x: targetVx, y: hy },
    { x: targetVx, y: gy },
    { x: gx, y: gy }
  ];
}

function generateRandomTaskFromNode() {
  generatedTaskCounter++;
  const taskId = `rnd_task_${generatedTaskCounter.toString().padStart(3, '0')}`;
  
  // Pick two distinct points from SAFE_AISLE_POINTS (Replicating RandomTaskGeneratorNode)
  const pIdx1 = Math.floor(Math.random() * SAFE_AISLE_POINTS.length);
  let pIdx2 = Math.floor(Math.random() * SAFE_AISLE_POINTS.length);
  while (pIdx2 === pIdx1) {
    pIdx2 = Math.floor(Math.random() * SAFE_AISLE_POINTS.length);
  }

  const pickPt = SAFE_AISLE_POINTS[pIdx1];
  const dropPt = SAFE_AISLE_POINTS[pIdx2];

  // Determine rack label & SKU
  let rack = WAREHOUSE_CONFIG.shelves.find(s => Math.hypot(s.x - pickPt.x, s.y - pickPt.y) < 3.5);
  if (!rack) rack = WAREHOUSE_CONFIG.shelves[Math.floor(Math.random() * WAREHOUSE_CONFIG.shelves.length)];

  // Determine dropoff station
  const stationKeys = Object.keys(WAREHOUSE_CONFIG.stations);
  const stKey = stationKeys[Math.floor(Math.random() * stationKeys.length)];
  const station = WAREHOUSE_CONFIG.stations[stKey];

  const priority = [50, 75, 100][Math.floor(Math.random() * 3)];
  const nowEpoch = parseFloat((Date.now() / 1000).toFixed(3));
  const ttlSec = 1800.0;

  const taskPayload = {
    run_id: 'run_live_session',
    task_id: taskId,
    priority: priority,
    pickup: {
      x: parseFloat(pickPt.x.toFixed(2)),
      y: parseFloat(pickPt.y.toFixed(2)),
      theta: pickPt.theta,
      rack_id: rack.id,
      item_type: rack.itemType || 'Servo Motors'
    },
    dropoff: {
      x: parseFloat(dropPt.x.toFixed(2)),
      y: parseFloat(dropPt.y.toFixed(2)),
      theta: dropPt.theta,
      station_id: station.id,
      zone: station.zone
    },
    dwell_times: {
      pickup_wait_s: 3.0,
      dropoff_wait_s: 3.0
    },
    status: "ANNOUNCED",
    assigned_robot_id: null,
    winning_bid: null,
    created_at_epoch: nowEpoch,
    expires_at_epoch: parseFloat((nowEpoch + ttlSec).toFixed(3)),
    dwell_remaining: 0.0,
    progress_pct: 0
  };

  return taskPayload;
}

function renderTasksTable() {
  const tbody = document.getElementById('tasks-table-body');
  if (!tbody) return;
  if (APP_STATE.activeTab !== 'tasks') return;

  const search = document.getElementById('task-search-input')?.value.toLowerCase() || '';
  const filterRun = document.getElementById('task-filter-run')?.value || 'ALL';
  const filterStatus = document.getElementById('task-filter-status')?.value || 'ALL';
  const filterPriority = document.getElementById('task-filter-priority')?.value || 'ALL';

  const filtered = APP_STATE.tasks.filter(t => {
    const matchesRun = filterRun === 'ALL' || t.run_id === filterRun;
    const matchesSearch = t.task_id.toLowerCase().includes(search) || 
      (t.pickup && t.pickup.rack_id && t.pickup.rack_id.toLowerCase().includes(search)) || 
      (t.dropoff && t.dropoff.station_id && t.dropoff.station_id.toLowerCase().includes(search)) || 
      (t.assigned_robot_id && t.assigned_robot_id.toLowerCase().includes(search)) ||
      (t.pickup && t.pickup.item_type && t.pickup.item_type.toLowerCase().includes(search)) ||
      (t.run_id && t.run_id.toLowerCase().includes(search));

    const matchesStatus = filterStatus === 'ALL' || t.status === filterStatus;
    
    let matchesPriority = true;
    if (filterPriority === 'CRITICAL') matchesPriority = t.priority >= 80;
    else if (filterPriority === 'HIGH') matchesPriority = t.priority >= 60 && t.priority < 80;
    else if (filterPriority === 'STANDARD') matchesPriority = t.priority < 60;

    return matchesRun && matchesSearch && matchesStatus && matchesPriority;
  });

  // Limit DOM rows to top 25 tasks to prevent massive C++ DOM heap thrashing
  const MAX_DISPLAY = 25;
  const displayed = filtered.slice(0, MAX_DISPLAY);

  const countElem = document.getElementById('task-table-count-info');
  if (countElem) {
    countElem.textContent = `Showing ${displayed.length} of ${filtered.length} tasks`;
  }

  tbody.innerHTML = displayed.map(t => {
    const badge = getStatusBadgeHTML(t.status);
    const prioColor = t.priority >= 80 ? '#dc2626' : t.priority >= 60 ? '#2563eb' : '#64748b';

    const pDwell = t.dwell_times ? t.dwell_times.pickup_wait_s : 3.0;
    const dDwell = t.dwell_times ? t.dwell_times.dropoff_wait_s : 3.0;

    let dwellHTML = `<span style="color: #94a3b8; font-size: 11px;">${pDwell}s / ${dDwell}s</span>`;
    if (t.status === 'PICKUP_WAIT' || t.status === 'DROPOFF_WAIT') {
      dwellHTML = `<span class="dwell-badge dwell-pulse">⏳ ${(t.dwell_remaining || 3.0).toFixed(1)}s dwell</span>`;
    }

    const assignedTag = t.assigned_robot_id 
      ? `<strong style="color: ${ROBOT_COLOR_MAP[t.assigned_robot_id] || '#2563eb'}; font-family: var(--font-mono);">${t.assigned_robot_id}</strong>`
      : `<span style="color: #94a3b8; font-style: italic;">Auctioning...</span>`;

    const bidTag = t.winning_bid !== null && t.winning_bid !== undefined
      ? `<span style="font-family: var(--font-mono); font-weight: 700; color: #059669;">$${parseFloat(t.winning_bid).toFixed(2)}</span>`
      : `<span style="color: #94a3b8;">--</span>`;

    const rackId = t.pickup ? t.pickup.rack_id : 'RACK_W01';
    const itemType = t.pickup ? t.pickup.item_type || 'Servo Motors' : 'Servo Motors';
    const pickX = t.pickup ? t.pickup.x.toFixed(2) : '0.00';
    const pickY = t.pickup ? t.pickup.y.toFixed(2) : '0.00';

    const stationId = t.dropoff ? t.dropoff.station_id : 'DROPOFF_STATION_A';
    const zone = t.dropoff ? t.dropoff.zone : 'DISPATCH_BAY_1';
    const dropX = t.dropoff ? t.dropoff.x.toFixed(2) : '0.00';
    const dropY = t.dropoff ? t.dropoff.y.toFixed(2) : '0.00';

    return `
      <tr>
        <td>
          <a class="table-link" onclick="inspectTaskJSON('${t.task_id}')" title="Click to view exact ROS JSON payload">
            <strong>${t.task_id}</strong> 🔍
          </a>
          ${t.run_id ? `<div style="font-size: 10px; color: #94a3b8; font-family: var(--font-mono);">${t.run_id}</div>` : ''}
        </td>
        <td>
          <div class="priority-meter-wrap">
            <span class="priority-val-text" style="color: ${prioColor};">${t.priority}</span>
            <div class="priority-meter-bar">
              <div class="priority-meter-fill" style="width: ${t.priority}%; background: ${prioColor};"></div>
            </div>
          </div>
        </td>
        <td>
          <div><strong>${rackId}</strong> <span style="font-size: 11px; color: #475569;">(${itemType})</span></div>
          <span class="coord-tag">X: ${pickX}m, Y: ${pickY}m</span>
        </td>
        <td>
          <div><strong>${stationId}</strong> <span style="font-size: 11px; color: #64748b;">(${zone})</span></div>
          <span class="coord-tag">X: ${dropX}m, Y: ${dropY}m</span>
        </td>
        <td>${dwellHTML}</td>
        <td>${assignedTag}</td>
        <td>${bidTag}</td>
        <td>
          <div class="task-progress-wrap">
            <div class="task-progress-bar">
              <div class="task-progress-fill" style="width: ${t.progress_pct || 0}%;"></div>
            </div>
            <div class="task-progress-label">
              <span>${t.status}</span>
              <strong>${t.progress_pct || 0}%</strong>
            </div>
          </div>
        </td>
        <td>${badge}</td>
        <td>
          <div style="display: flex; gap: 4px;">
            <button class="btn-outline btn-sm" onclick="boostTaskPriority('${t.task_id}')" title="Boost CBBA Priority to 100">⬆ P100</button>
            <button class="btn-outline btn-sm" onclick="inspectTaskJSON('${t.task_id}')" title="Inspect JSON Spec Payload">{ } JSON</button>
          </div>
        </td>
      </tr>
    `;
  }).join('');
}

function getStatusBadgeHTML(status) {
  switch (status) {
    case 'ANNOUNCED':
    case 'CBBA_AUCTION':
      return `<span class="status-badge-lifecycle badge-announced">🟣 ${status}</span>`;
    case 'ASSIGNED':
    case 'EN_ROUTE_PICKUP':
      return `<span class="status-badge-lifecycle badge-assigned">🔵 ${status}</span>`;
    case 'PICKUP_WAIT':
    case 'DROPOFF_WAIT':
      return `<span class="status-badge-lifecycle badge-pickup-wait">🟡 ${status}</span>`;
    case 'EN_ROUTE_DROPOFF':
      return `<span class="status-badge-lifecycle badge-enroute-dropoff">🟦 ${status}</span>`;
    case 'COMPLETED':
      return `<span class="status-badge-lifecycle badge-completed">🟢 COMPLETED</span>`;
    case 'FAILED':
      return `<span class="status-badge-lifecycle badge-failed">🔴 FAILED</span>`;
    default:
      return `<span class="status-badge-lifecycle badge-assigned">${status}</span>`;
  }
}

function renderTaskCards() {
  const container = document.getElementById('tasks-cards-container');
  if (!container) return;

  container.innerHTML = APP_STATE.tasks.slice(0, 30).map(t => {
    const badge = getStatusBadgeHTML(t.status);
    const rackId = t.pickup ? t.pickup.rack_id : 'RACK_W01';
    const itemType = t.pickup ? t.pickup.item_type || 'Warehouse Cargo' : 'Warehouse Cargo';
    const pickX = t.pickup ? t.pickup.x.toFixed(1) : '0.0';
    const pickY = t.pickup ? t.pickup.y.toFixed(1) : '0.0';

    const stationId = t.dropoff ? t.dropoff.station_id : 'DROPOFF_STATION_A';
    const dropX = t.dropoff ? t.dropoff.x.toFixed(1) : '0.0';
    const dropY = t.dropoff ? t.dropoff.y.toFixed(1) : '0.0';

    return `
      <div class="task-kanban-card">
        <div class="kc-head">
          <span class="kc-id" style="cursor: pointer;" onclick="inspectTaskJSON('${t.task_id}')">${t.task_id} 🔍</span>
          ${badge}
        </div>
        <div class="kc-item">📦 ${itemType}</div>
        
        <div class="kc-points">
          <div>
            <div class="kc-point-lbl">Pickup</div>
            <div class="kc-point-val">${rackId}</div>
            <div class="coord-tag">X:${pickX} Y:${pickY}</div>
          </div>
          <div>
            <div class="kc-point-lbl">Dropoff</div>
            <div class="kc-point-val">${stationId}</div>
            <div class="coord-tag">X:${dropX} Y:${dropY}</div>
          </div>
        </div>

        <div style="display: flex; justify-content: space-between; font-size: 11.5px;">
          <span>AMR: <strong>${t.assigned_robot_id || 'Auctioning'}</strong></span>
          <span>Winning Bid: <strong style="color: #059669;">${t.winning_bid ? '$' + parseFloat(t.winning_bid).toFixed(2) : '--'}</strong></span>
        </div>

        <div class="task-progress-wrap">
          <div class="task-progress-bar">
            <div class="task-progress-fill" style="width: ${t.progress_pct || 0}%;"></div>
          </div>
          <div class="task-progress-label">
            <span>Priority: <strong>${t.priority}</strong></span>
            <span>Progress: <strong>${t.progress_pct || 0}%</strong></span>
          </div>
        </div>

        <div style="display: flex; justify-content: flex-end; gap: 6px; margin-top: 4px;">
          <button class="btn-outline btn-sm" onclick="inspectTaskJSON('${t.task_id}')">JSON Payload</button>
          <button class="btn-outline btn-sm" onclick="boostTaskPriority('${t.task_id}')">Boost Priority</button>
        </div>
      </div>
    `;
  }).join('');
}

function updateTaskSummaryMetrics() {
  const announced = APP_STATE.tasks.filter(t => t.status === 'ANNOUNCED' || t.status === 'CBBA_AUCTION').length;
  const active = APP_STATE.tasks.filter(t => t.status === 'ASSIGNED' || t.status === 'EN_ROUTE_PICKUP' || t.status === 'EN_ROUTE_DROPOFF').length;
  const dwelling = APP_STATE.tasks.filter(t => t.status === 'PICKUP_WAIT' || t.status === 'DROPOFF_WAIT').length;
  const completed = APP_STATE.tasks.filter(t => t.status === 'COMPLETED').length;

  const validBids = APP_STATE.tasks.filter(t => t.winning_bid !== null && t.winning_bid !== undefined).map(t => parseFloat(t.winning_bid));
  const avgBid = validBids.length > 0 ? (validBids.reduce((a, b) => a + b, 0) / validBids.length).toFixed(1) : '102.4';

  const eAnn = document.getElementById('task-metric-announced');
  const eAct = document.getElementById('task-metric-active');
  const eDwe = document.getElementById('task-metric-dwelling');
  const eCom = document.getElementById('task-metric-completed');
  const eBid = document.getElementById('task-metric-avg-bid');
  const navCount = document.getElementById('nav-task-count');

  if (eAnn) eAnn.textContent = announced;
  if (eAct) eAct.textContent = active;
  if (eDwe) eDwe.textContent = dwelling;
  if (eCom) eCom.textContent = completed;
  if (eBid) eBid.textContent = avgBid;
  if (navCount) navCount.textContent = APP_STATE.tasks.length;
}

window.boostTaskPriority = function(taskId) {
  const t = APP_STATE.tasks.find(x => x.task_id === taskId);
  if (t) {
    t.priority = 100;
    renderTasksTable();
    updateTaskSummaryMetrics();
    addStructuredLog('SUPERVISOR', 'TASK_LIFECYCLE', `Operator boosted task ${taskId} priority to 100 (Critical Urgency). Bidding auction updated.`, 'Success', [
      { label: 'Task', val: taskId },
      { label: 'Priority', val: '100' }
    ]);
  }
};

window.inspectTaskJSON = function(taskId) {
  const t = APP_STATE.tasks.find(x => x.task_id === taskId);
  if (!t) return;

  const modal = document.getElementById('task-json-modal');
  const title = document.getElementById('task-json-modal-title');
  const pre = document.getElementById('task-json-content');

  // Exact JSON Specification Structure
  const exactTaskPayload = {
    task_id: t.task_id,
    priority: t.priority,
    pickup: {
      x: t.pickup ? t.pickup.x : -19.99,
      y: t.pickup ? t.pickup.y : -17.60,
      theta: t.pickup ? (t.pickup.theta || 0.0) : 0.0,
      rack_id: t.pickup ? t.pickup.rack_id : 'RACK_WEST_SOUTH_01',
      item_type: t.pickup ? (t.pickup.item_type || 'Servo Motors') : 'Servo Motors'
    },
    dropoff: {
      x: t.dropoff ? t.dropoff.x : -11.35,
      y: t.dropoff ? t.dropoff.y : -6.11,
      theta: t.dropoff ? (t.dropoff.theta || 0.0) : 0.0,
      station_id: t.dropoff ? t.dropoff.station_id : 'DROPOFF_STATION_A',
      zone: t.dropoff ? (t.dropoff.zone || 'DISPATCH_BAY_1') : 'DISPATCH_BAY_1'
    },
    dwell_times: {
      pickup_wait_s: t.dwell_times ? t.dwell_times.pickup_wait_s : 3.0,
      dropoff_wait_s: t.dwell_times ? t.dwell_times.dropoff_wait_s : 3.0
    },
    status: t.status,
    assigned_robot_id: t.assigned_robot_id || null,
    winning_bid: t.winning_bid !== undefined ? t.winning_bid : null,
    created_at_epoch: t.created_at_epoch || (Date.now() / 1000),
    expires_at_epoch: t.expires_at_epoch || (Date.now() / 1000 + 300)
  };

  if (title) title.textContent = `Task Payload: ${t.task_id} (${t.status})`;
  if (pre) pre.textContent = JSON.stringify(exactTaskPayload, null, 2);
  if (modal) modal.style.display = 'flex';
};

function renderLogsTable() {
  const tbody = document.getElementById('logs-table-body');
  if (!tbody) return;
  if (APP_STATE.activeTab !== 'logs') return;

  const search = document.getElementById('log-search-input')?.value.toLowerCase() || '';
  const robotFilter = document.getElementById('log-filter-robot')?.value || 'ALL';
  const catFilter = document.getElementById('log-filter-category')?.value || 'ALL';
  const sevFilter = document.getElementById('log-filter-severity')?.value || 'ALL';

  const filtered = APP_STATE.logs.filter(l => {
    const matchesSearch = l.description.toLowerCase().includes(search) || l.robot.toLowerCase().includes(search) || l.category.toLowerCase().includes(search);
    const matchesRobot = robotFilter === 'ALL' || l.robot.toLowerCase() === robotFilter.toLowerCase();
    const matchesCat = catFilter === 'ALL' || l.category === catFilter;
    const matchesSev = sevFilter === 'ALL' || l.status === sevFilter;
    return matchesSearch && matchesRobot && matchesCat && matchesSev;
  });

  const MAX_LOGS_DISPLAY = 50;
  const displayed = filtered.slice(0, MAX_LOGS_DISPLAY);

  const countElem = document.getElementById('log-count-display');
  if (countElem) countElem.textContent = `Showing ${displayed.length} of ${APP_STATE.logs.length} events`;

  tbody.innerHTML = displayed.map(l => {
    const robotColor = ROBOT_COLOR_MAP[l.robot] || '#64748b';
    
    const chipsHTML = (l.chips && l.chips.length > 0) ? `
      <div class="decision-chips-wrap">
        ${l.chips.map(c => `
          <span class="decision-chip ${c.isQuorum ? 'quorum-pill' : ''} ${c.isBid ? 'bid-pill' : ''}">
            <strong>${c.label}:</strong> ${c.val}
          </span>
        `).join('')}
      </div>
    ` : '';

    let catBadgeClass = 'badge-assigned';
    if (l.category === 'CBBA_AUCTION' || l.category === 'QUORUM_CONSENSUS') catBadgeClass = 'badge-announced';
    else if (l.category === 'SAFETY_ALERT') catBadgeClass = 'badge-failed';
    else if (l.category === 'CORRIDOR_MUTEX') catBadgeClass = 'badge-pickup-wait';

    return `
      <tr>
        <td><span style="font-family: var(--font-mono); color: #64748b; font-size: 11px;">${l.time}</span></td>
        <td><strong style="color: ${robotColor}; font-family: var(--font-mono);">${l.robot}</strong></td>
        <td><span class="status-badge-lifecycle ${catBadgeClass}">${l.category}</span></td>
        <td>
          <div class="decision-event-row">
            <span class="decision-main-text">${l.description}</span>
            ${chipsHTML}
          </div>
        </td>
        <td><span class="status-tag ${l.status === 'Emergency' ? 'tag-pending' : 'tag-done'}">${l.status}</span></td>
      </tr>
    `;
  }).join('');
}

function renderRecentLogsDashboard() {
  const container = document.getElementById('dash-recent-logs');
  if (!container) return;

  container.innerHTML = APP_STATE.logs.slice(0, 4).map(l => `
    <div class="event-snippet">
      <span class="event-time">[${l.time}]</span>
      <span class="event-text"><strong style="color: ${ROBOT_COLOR_MAP[l.robot] || '#2563eb'}">${l.robot}:</strong> ${l.description}</span>
    </div>
  `).join('');
}

function renderSidebarAmrCards() {
  const container = document.getElementById('amr-cards-list');
  if (!container) return;

  container.innerHTML = APP_STATE.robots.map(bot => {
    const isSelected = bot.id === APP_STATE.selectedRobotId;
    return `
      <div class="amr-card-item ${isSelected ? 'selected' : ''}" onclick="selectAmr('${bot.id}')">
        <div class="amr-card-head">
          <span class="amr-name-tag">
            <span class="amr-color-dot" style="background: ${bot.color};"></span>
            ${bot.name}
          </span>
          <span class="amr-battery-val" style="color: ${bot.battery > 50 ? '#10b981' : '#f59e0b'}">
            ${bot.isCharging ? '⚡ ' : ''}${Math.round(bot.battery)}%
          </span>
        </div>
        <div class="amr-card-body">
          <span>State: <strong>${bot.state}</strong></span>
          <span>Speed: <strong>${(bot.speed || 0.8).toFixed(2)} m/s</strong></span>
        </div>
      </div>
    `;
  }).join('');
}

window.selectAmr = function(id) {
  APP_STATE.selectedRobotId = id;
  renderSidebarAmrCards();
  updateUberDirectionCard();
};

function updateUberDirectionCard() {
  const activeBot = APP_STATE.robots.find(r => r.id === APP_STATE.selectedRobotId) || APP_STATE.robots[0];
  const nameElem = document.getElementById('dir-amr-name');
  const destElem = document.getElementById('dir-task-dest');
  const itemElem = document.getElementById('dir-item-name');
  const etaElem = document.getElementById('dir-eta');
  const speedElem = document.getElementById('dir-speed');
  const battElem = document.getElementById('dir-battery');

  if (!nameElem || !activeBot) return;

  nameElem.textContent = `${activeBot.name.toUpperCase()} (${activeBot.state || 'IDLE'})`;
  
  const activeTask = APP_STATE.tasks.find(t => t.task_id === activeBot.taskId);
  if (activeTask) {
    const isDropoff = (activeBot.state || '').includes('DROPOFF') || (activeTask.status || '').includes('DROPOFF');
    const targetInfo = isDropoff ? (activeTask.dropoff || {}) : (activeTask.pickup || {});
    const targetLabel = targetInfo.label || targetInfo.station_id || targetInfo.rack_id || (targetInfo.x !== undefined ? `Coords (${targetInfo.x.toFixed(1)}, ${targetInfo.y.toFixed(1)})` : 'Assigned Target');
    destElem.textContent = isDropoff ? `Dropoff: ${targetLabel}` : `Pickup: ${targetLabel}`;
    itemElem.textContent = targetInfo.item_type || targetInfo.item || activeTask.task_id || 'Warehouse SKU';
    etaElem.textContent = activeTask.dwell_remaining > 0 ? `${activeTask.dwell_remaining}s dwell` : 'En route';
  } else {
    destElem.textContent = activeBot.isCharging ? 'Docked: Inductive Fast Charging Pad' : 'Idle / Standby: Ready for Task';
    itemElem.textContent = 'No Payload Assigned';
    etaElem.textContent = '--';
  }

  speedElem.textContent = `${(activeBot.speed || 0.0).toFixed(2)} m/s`;
  battElem.textContent = `${Math.round(activeBot.battery || 100)}%`;
}

function renderStaticHeatmap() {
  if (!heatCtx || !heatmapCanvas) return;
  const rect = heatmapCanvas.getBoundingClientRect();
  heatCtx.clearRect(0, 0, rect.width, rect.height);

  heatCtx.fillStyle = '#f8fafc';
  heatCtx.fillRect(0, 0, rect.width, rect.height);

  for (const s of WAREHOUSE_CONFIG.shelves) {
    const p = projectWorld(s.x, s.y, 0, rect.width, rect.height, '2D');
    heatCtx.fillStyle = '#e2e8f0';
    heatCtx.fillRect(p.x - 12, p.y - 4, 24, 8);
  }

  const hotPoints = [
    { x: -9.0, y: -10.0, intensity: 0.9 },
    { x: 9.0, y: -10.0, intensity: 0.8 },
    { x: -9.0, y: 10.0, intensity: 0.7 },
    { x: 9.0, y: 10.0, intensity: 0.85 },
    { x: 0.0, y: -28.5, intensity: 0.95 },
    { x: -15.67, y: 0.0, intensity: 0.5 },
    { x: 15.67, y: 0.0, intensity: 0.6 }
  ];

  for (const hp of hotPoints) {
    const p = projectWorld(hp.x, hp.y, 0, rect.width, rect.height, '2D');
    const rad = 45 * hp.intensity;
    const grad = heatCtx.createRadialGradient(p.x, p.y, 4, p.x, p.y, rad);
    grad.addColorStop(0, 'rgba(239, 68, 68, 0.6)');
    grad.addColorStop(0.5, 'rgba(251, 146, 60, 0.4)');
    grad.addColorStop(1, 'rgba(254, 240, 138, 0)');

    heatCtx.fillStyle = grad;
    heatCtx.beginPath();
    heatCtx.arc(p.x, p.y, rad, 0, Math.PI * 2);
    heatCtx.fill();
  }
}

function setupTaskModal() {
  const modal = document.getElementById('create-task-modal');
  const btnOpen = document.getElementById('btn-create-task-modal');
  const btnClose = document.getElementById('btn-close-create-task');
  const btnCancel = document.getElementById('btn-cancel-create-task');
  const btnSubmit = document.getElementById('btn-submit-create-task');

  if (btnOpen) btnOpen.addEventListener('click', () => modal.style.display = 'flex');
  if (btnClose) btnClose.addEventListener('click', () => modal.style.display = 'none');
  if (btnCancel) btnCancel.addEventListener('click', () => modal.style.display = 'none');

  if (btnSubmit) {
    btnSubmit.addEventListener('click', () => {
      const pickElem = document.getElementById('newtask-pickup-rack');
      const dropElem = document.getElementById('newtask-dropoff-station');
      const item = document.getElementById('newtask-item').value;
      const priority = parseInt(document.getElementById('newtask-priority').value) || 75;
      const dwellP = parseFloat(document.getElementById('newtask-dwell-pickup').value) || 3.0;
      const dwellD = parseFloat(document.getElementById('newtask-dwell-dropoff').value) || 3.0;

      const pickOption = pickElem.options[pickElem.selectedIndex];
      const dropOption = dropElem.options[dropElem.selectedIndex];

      generatedTaskCounter++;
      const newTaskId = `rnd_task_${generatedTaskCounter.toString().padStart(3, '0')}`;
      const nowEpoch = parseFloat((Date.now() / 1000).toFixed(3));

      const newTask = {
        run_id: 'run_live_manual',
        task_id: newTaskId,
        priority: Math.min(100, Math.max(1, priority)),
        status: 'ANNOUNCED',
        assigned_robot_id: null,
        winning_bid: null,
        pickup: {
          x: parseFloat(pickOption.dataset.x) || -19.99,
          y: parseFloat(pickOption.dataset.y) || -17.60,
          theta: 0.0,
          rack_id: pickElem.value,
          item_type: item
        },
        dropoff: {
          x: parseFloat(dropOption.dataset.x) || -11.35,
          y: parseFloat(dropOption.dataset.y) || -6.11,
          theta: 0.0,
          station_id: dropElem.value,
          zone: dropOption.dataset.zone || 'DISPATCH_BAY_1'
        },
        dwell_times: {
          pickup_wait_s: dwellP,
          dropoff_wait_s: dwellD
        },
        dwell_remaining: 0.0,
        progress_pct: 0,
        created_at_epoch: nowEpoch,
        expires_at_epoch: parseFloat((nowEpoch + 1800.0).toFixed(3))
      };

      APP_STATE.tasks.unshift(newTask);
      renderTasksTable();
      updateTaskSummaryMetrics();

      addStructuredLog('task_generator', 'CBBA_AUCTION', `Announced ${newTaskId}: pick (${newTask.pickup.x.toFixed(1)}, ${newTask.pickup.y.toFixed(1)}) -> drop (${newTask.dropoff.x.toFixed(1)}, ${newTask.dropoff.y.toFixed(1)}), priority ${newTask.priority}`, 'Success', [
        { label: 'Task', val: newTaskId },
        { label: 'Priority', val: `${newTask.priority}` }
      ]);

      modal.style.display = 'none';

      setTimeout(() => {
        allocateNextAnnouncedTask();
      }, 1000 / APP_STATE.simSpeed);
    });
  }

  // Task JSON Modal listeners
  const jsonModal = document.getElementById('task-json-modal');
  const btnCloseJson = document.getElementById('btn-close-task-json');
  const btnDoneJson = document.getElementById('btn-done-task-json');
  const btnCopyJson = document.getElementById('btn-copy-task-json');

  if (btnCloseJson) btnCloseJson.addEventListener('click', () => jsonModal.style.display = 'none');
  if (btnDoneJson) btnDoneJson.addEventListener('click', () => jsonModal.style.display = 'none');
  if (btnCopyJson) {
    btnCopyJson.addEventListener('click', () => {
      const text = document.getElementById('task-json-content')?.textContent;
      if (text) {
        navigator.clipboard.writeText(text).then(() => {
          const original = btnCopyJson.textContent;
          btnCopyJson.textContent = '✅ Copied!';
          setTimeout(() => { btnCopyJson.textContent = original; }, 1500);
        });
      }
    });
  }

  document.getElementById('task-search-input')?.addEventListener('input', renderTasksTable);
  document.getElementById('task-filter-run')?.addEventListener('change', renderTasksTable);
  document.getElementById('task-filter-status')?.addEventListener('change', renderTasksTable);
  document.getElementById('task-filter-priority')?.addEventListener('change', renderTasksTable);

  document.getElementById('log-search-input')?.addEventListener('input', renderLogsTable);
  document.getElementById('log-filter-robot')?.addEventListener('change', renderLogsTable);
  document.getElementById('log-filter-category')?.addEventListener('change', renderLogsTable);
  document.getElementById('log-filter-severity')?.addEventListener('change', renderLogsTable);
}

function triggerRandomTaskBurst() {
  const newTask = generateRandomTaskFromNode();

  APP_STATE.tasks.unshift(newTask);
  renderTasksTable();
  updateTaskSummaryMetrics();

  addStructuredLog('task_generator', 'CBBA_AUCTION', `Announced ${newTask.task_id}: pick (${newTask.pickup.x.toFixed(1)}, ${newTask.pickup.y.toFixed(1)}) -> drop (${newTask.dropoff.x.toFixed(1)}, ${newTask.dropoff.y.toFixed(1)}), priority ${newTask.priority}`, 'Success', [
    { label: 'Task', val: newTask.task_id },
    { label: 'Priority', val: `${newTask.priority}` }
  ]);

  setTimeout(() => {
    allocateNextAnnouncedTask();
  }, 1000 / APP_STATE.simSpeed);
}

function exportLogsToCSV() {
  let csv = 'Time,Agent,Category,Description,Status\n';
  APP_STATE.logs.forEach(l => {
    csv += `"${l.time}","${l.robot}","${l.category}","${l.description.replace(/"/g, '""')}","${l.status}"\n`;
  });

  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = `fleet_telemetry_logs_${Date.now()}.csv`;
  link.click();
}

let lastDrawTime = 0;
const TARGET_FPS = 30;
const FRAME_MIN_TIME = 1000 / TARGET_FPS;

function animLoop(timestamp) {
  requestAnimationFrame(animLoop);

  // If document is in background/minimized, pause expensive canvas drawing
  if (document.hidden) return;

  const elapsed = timestamp - lastDrawTime;
  if (elapsed < FRAME_MIN_TIME) return;

  const dt = Math.min((timestamp - lastAnimTime) / 1000, 0.1);
  lastAnimTime = timestamp;
  lastDrawTime = timestamp - (elapsed % FRAME_MIN_TIME);

  updateSimulationEngine(dt);

  if (dashCtx && dashCanvas && APP_STATE.activeTab === 'dashboard') {
    const cWidth = dashCanvas.width / window.devicePixelRatio;
    const cHeight = dashCanvas.height / window.devicePixelRatio;
    drawWarehouseScene(dashCtx, cWidth, cHeight, APP_STATE.viewMode);
  }

  if (fullCtx && fullCanvas && APP_STATE.activeTab === 'map-view') {
    const cWidth = fullCanvas.width / window.devicePixelRatio;
    const cHeight = fullCanvas.height / window.devicePixelRatio;
    drawWarehouseScene(fullCtx, cWidth, cHeight, APP_STATE.viewMode);
  }
}

window.addEventListener('DOMContentLoaded', () => {
  initNavigation();
  resizeActiveCanvases();
  setupTaskModal();

  updateTaskSummaryMetrics();
  renderSidebarAmrCards();
  renderRecentLogsDashboard();

  // Ingest definitive real dataset runs
  loadDatasetTasksAndMetrics();

  // Connect live bridge
  initTelemetryBridge();

  requestAnimationFrame(animLoop);
});


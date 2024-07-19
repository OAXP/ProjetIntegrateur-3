export const SERVER_BASE_URL = 'http://localhost:3000/';
export const SERVER_API_URL = 'http://localhost:3000/api';

// Battery voltage constants
export const MIN_VOLTAGE = 8.25;
export const MAX_VOLTAGE = 12.6;

export const DEFAULT_DASHBOARD_LAYOUT = [
    { cols: 2, rows: 3, y: 4, x: 0 }, // System-runtime
    { cols: 2, rows: 4, y: 0, x: 0 }, // System-info
    { cols: 3, rows: 5, y: 2, x: 4 }, // XTerm
    { cols: 2, rows: 3, y: 4, x: 2 }, // Locate
    { cols: 3, rows: 4, y: 0, x: 7 }, // Mission-history
    { cols: 3, rows: 2, y: 0, x: 4 }, // Mission-control
    { cols: 2, rows: 4, y: 0, x: 2 }, // Map
    { cols: 3, rows: 3, y: 4, x: 7 }, // Init-position
];
import { Coordinates } from "./Coordinates";
import { Quaternion } from "./Quaternion";

export interface RobotInfo {
    robot_id: string;
    status: string;
    battery_voltage: number;
    start_time: number;
    position_x: number;
    position_y: number;
    init_position: Coordinates;
    init_orientation: Quaternion;
}
import Logger from "@src/utils/Logger";
import * as rclnodejs from 'rclnodejs';
import { Service } from "typedi";
import { RosService } from "./ros.service";
import { FileService } from "./file.service";
import { SocketManagerService } from "./socket-manager.service";
import { Coordinates } from "@src/interfaces/Coordinates";
import { Quaternion } from "@src/interfaces/Quaternion";

interface RobotInitPositions {
    init_position: Coordinates;
    init_orientation: Quaternion;
}

type RobotInfo = rclnodejs.communication_interfaces.msg.RobotInfo & RobotInitPositions;

@Service()
export class InfoService {
    private readonly LOGGER: Logger;
    private robots: Map<string, RobotInfo> = new Map();

    constructor(private readonly rosService: RosService, private readonly socketManagerService: SocketManagerService, private readonly fileService: FileService) {
        this.LOGGER = new Logger('InfoService', fileService);
    }

    get robotInfo() {
        return this.robots;
    }

    init() {
        const node = this.rosService.getNode('info_sub_node');
        node.createSubscription('communication_interfaces/msg/RobotInfo', '/robot_info', (msg: rclnodejs.communication_interfaces.msg.RobotInfo) => {this.onRobotInfo(msg)});
    }

    async onRobotInfo(msg: rclnodejs.communication_interfaces.msg.RobotInfo) {
        const robotInitPositions = this.robots.get(msg.robot_id)?.init_position ?? { x: 0, y: 0 };
        const robotInitOrientation = this.robots.get(msg.robot_id)?.init_orientation ?? { x: 0, y: 0, z: 0, w: 1 };
        const robotInfo: RobotInfo = { ...msg, init_position: robotInitPositions, init_orientation: robotInitOrientation };
        this.robots.set(msg.robot_id, robotInfo);
        this.socketManagerService.sio.emit('robot_info', robotInfo);
        if (this.fileService.isExploring) 
            this.LOGGER.info(`${robotInfo.robot_id} info: Position: (${robotInfo.position_x.toFixed(2)}, ${robotInfo.position_y.toFixed(2)}) Distance: ${robotInfo.distance.toFixed(2)}`);
    }

    setInitPositions(initPositions: { [id: string]: { init_position: Coordinates; init_orientation: Quaternion } }) {
        this.robots.forEach((robot, robot_id) => {
            if (initPositions[robot_id]) {
                robot.init_position = initPositions[robot_id].init_position;
                robot.init_orientation = initPositions[robot_id].init_orientation;
            }
        });
    }
}
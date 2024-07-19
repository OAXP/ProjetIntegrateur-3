import { Service } from 'typedi';
import * as rclnodejs from 'rclnodejs';
import { SocketManagerService } from "./socket-manager.service";
import { RosService } from './ros.service';
import { createCanvas } from 'canvas';
import Logger from '@src/utils/Logger';
import { FileService } from './file.service';
import { InfoService } from './info.service';
@Service()
export class MapService {
  private readonly LOGGER: Logger;
  private canvas: any;
  private ctx: any;
  private map1: rclnodejs.nav_msgs.msg.OccupancyGrid;
  private map2: rclnodejs.nav_msgs.msg.OccupancyGrid;

  constructor(private readonly rosService: RosService, private readonly socketManagerService: SocketManagerService, private readonly fileService: FileService, private readonly infoService: InfoService) {
    this.canvas = createCanvas(0, 0);
    this.ctx = this.canvas.getContext('2d');
    this.LOGGER = new Logger('MapService', fileService);
  }

  init() {
    const node = this.rosService.getNode('map_to_image_node');
    node.createSubscription('nav_msgs/msg/OccupancyGrid', 'lm1342/map', (msg: rclnodejs.nav_msgs.msg.OccupancyGrid) => this.map1_callback(msg));
    node.createSubscription('nav_msgs/msg/OccupancyGrid', 'lm1166/map', (msg: rclnodejs.nav_msgs.msg.OccupancyGrid) => this.map2_callback(msg));
    node.createSubscription('nav_msgs/msg/OccupancyGrid', 'limo_0/limo/map', (msg: rclnodejs.nav_msgs.msg.OccupancyGrid) => this.map1_callback(msg));
    node.createSubscription('nav_msgs/msg/OccupancyGrid', 'limo_1/limo/map', (msg: rclnodejs.nav_msgs.msg.OccupancyGrid) => this.map2_callback(msg));
  }

  clearCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  private map1_callback(msg: rclnodejs.nav_msgs.msg.OccupancyGrid) {
    this.map1 = msg;
    if (this.map2) {
      const mergedMap = this.mergeMap(this.map1, this.map2);
      this.transmitMap(mergedMap);
    } else {
      this.transmitMap(msg)
    }
  }

  private map2_callback(msg: rclnodejs.nav_msgs.msg.OccupancyGrid) {
    this.map2 = msg;
    if (this.map1) {
      const mergedMap = this.mergeMap(this.map1, this.map2);
      this.transmitMap(mergedMap);
    } else {
      this.transmitMap(msg)
    }
  }

  private mergeMap(map1: rclnodejs.nav_msgs.msg.OccupancyGrid, map2: rclnodejs.nav_msgs.msg.OccupancyGrid) {
    const mergedMap: rclnodejs.nav_msgs.msg.OccupancyGrid = rclnodejs.createMessageObject('nav_msgs/msg/OccupancyGrid');
    mergedMap.header = map1.header;
    mergedMap.header.frame_id = 'merge_map';

    const minX = Math.min(map1.info.origin.position.x, map2.info.origin.position.x);
    const minY = Math.min(map1.info.origin.position.y, map2.info.origin.position.y);
    const maxX = Math.max(map1.info.origin.position.x + (map1.info.width * map1.info.resolution),
                          map2.info.origin.position.x + (map2.info.width * map2.info.resolution));
    const maxY = Math.max(map1.info.origin.position.y + (map1.info.height * map1.info.resolution),
                          map2.info.origin.position.y + (map2.info.height * map2.info.resolution));
    
    mergedMap.info.origin.position.x = minX;
    mergedMap.info.origin.position.y = minY;
    mergedMap.info.resolution = Math.min(map1.info.resolution, map2.info.resolution);
    mergedMap.info.width = Math.ceil((maxX - minX) / mergedMap.info.resolution);
    mergedMap.info.height = Math.ceil((maxY - minY) / mergedMap.info.resolution);
    mergedMap.data = new Array(mergedMap.info.width * mergedMap.info.height).fill(-1);

    for (let y = 0; y < map1.info.height; y++) {
      for (let x = 0; x < map1.info.width; x++) {
        const value = map1.data[x + y * map1.info.width];
        const newX = Math.floor((map1.info.origin.position.x + x * map1.info.resolution - minX) / mergedMap.info.resolution);
        const newY = Math.floor((map1.info.origin.position.y + y * map1.info.resolution - minY) / mergedMap.info.resolution);
        const index = newX + newY * mergedMap.info.width;
        mergedMap.data[index] = value;
      }
    }

    for (let y = 0; y < map2.info.height; y++) {
      for (let x = 0; x < map2.info.width; x++) {
        const value = map2.data[x + y * map2.info.width];
        const newX = Math.floor((map2.info.origin.position.x + x * map2.info.resolution - minX) / mergedMap.info.resolution);
        const newY = Math.floor((map2.info.origin.position.y + y * map2.info.resolution - minY) / mergedMap.info.resolution);
        const index = newX + newY * mergedMap.info.width;
        if (mergedMap.data[index] === -1) {
          mergedMap.data[index] = value;
        }
      }
    }

    return mergedMap;
  }

  private transmitMap(msg: rclnodejs.nav_msgs.msg.OccupancyGrid): void {
    if (this.fileService.isExploring) {
        this.canvas.width = Math.max(this.canvas.width, msg.info.width);
        this.canvas.height = Math.max(this.canvas.height, msg.info.height);
        let scaleFactorX = this.canvas.width / msg.info.width;
        let scaleFactorY = this.canvas.height / msg.info.height;

        for (let y = 0; y < msg.info.height; y++) {
            for (let x = 0; x < msg.info.width; x++) {
                const value = msg.data[y * msg.info.width + x];
                let color: [number, number, number, number];
                if (value >= 70) color = [0, 0, 0, 255];
                else if (value >= 0) color = [255, 255, 255, 25];
                else color = [127, 127, 127, 25];
                this.ctx.fillStyle = `rgba(${color.join(',')})`;

                this.ctx.fillRect(x * scaleFactorX, (msg.info.height - y - 1) * scaleFactorY, scaleFactorX, scaleFactorY);
            }
        }
        
        const robotPositions = Array.from(this.infoService.robotInfo.values());

        const dotSize = 5;
        this.ctx.fillStyle = 'rgba(255, 0, 0, 255)'; 

        robotPositions.forEach(robot => {
            const canvasRobotX = this.canvas.width / 2 + scaleFactorX * robot.position_x * 20;
            const canvasRobotY = this.canvas.height / 2 - scaleFactorY * robot.position_y * 20;

            this.ctx.fillRect(canvasRobotX - dotSize / 2, canvasRobotY - dotSize / 2, dotSize, dotSize);
        });

        this.canvas.toBuffer((err, buf) => {
            if (err) {
                this.LOGGER.err('Error converting canvas to buffer: ', err);
                return;
            }
            this.socketManagerService.sio.emit('map', buf);
        });
    }
}
}

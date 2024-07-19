import Logger from '@src/utils/Logger';
import * as rclnodejs from 'rclnodejs';
import { Service } from 'typedi';
import { RosService } from './ros.service';
import { MapService } from './map.service';
import { FileService } from './file.service';
import { InfoService } from './info.service';
import { SocketManagerService } from './socket-manager.service';

@Service()
export class CommandService {
  private readonly LOGGER: Logger;
  private missionStarted = false;

  constructor(
    private readonly rosService: RosService,
    private readonly mapService: MapService,
    private readonly fileService: FileService,
    private readonly infoService: InfoService,
    private readonly socketService: SocketManagerService
  ) {
    this.LOGGER = new Logger("CommandService", fileService);
  }

  async identify(robot_id = "all") {
    if (!this.rosService.initialized) {
      this.LOGGER.err(
        "ROS context is not initialized. Skipping identification."
      );
      return;
    }
    try {
      const node = this.rosService.getNode("identify_pub_node");
      const msg: rclnodejs.communication_interfaces.msg.RobotIdentification = {
        robot_id,
      };
      msg.robot_id = robot_id;

      const publisher = node.createPublisher(
        "communication_interfaces/msg/RobotIdentification",
        "/identification"
      );
      publisher.publish(msg);
      node.destroyPublisher(publisher);

      this.LOGGER.info(
        `Identification message published for robot_id: ${robot_id}`
      );
    } catch (error) {
      this.LOGGER.err("Error identifying robot:", error);
    }
  }

  async startMission() {
    if (!this.rosService.initialized) {
      this.LOGGER.err(
        "ROS context is not initialized. Skipping identification."
      );
      return;
    }

    try {
      if (!this.missionStarted) {
        this.missionStarted = true;
        await this.fileService.startExploration(this.infoService.robotInfo);
        this.socketService.sendLastMission();
      }
      this.mapService.clearCanvas();
      const node = this.rosService.getNode("start_mission_pub_node");

      const msg: rclnodejs.std_msgs.msg.Bool = { data: true };

      const publisher = node.createPublisher(
        "std_msgs/msg/Bool",
        "/exploration"
      );
      publisher.publish(msg);
      node.destroyPublisher(publisher);

      this.LOGGER.info("Start mission message published");
    } catch (error) {
      this.LOGGER.err("Error starting mission:", error);
    }
  }

  stopMission() {
    if (!this.rosService.initialized) {
      this.LOGGER.err(
        "ROS context is not initialized. Skipping identification."
      );
      return;
    }
    try {
      this.fileService.endExploration();
      if (this.missionStarted) {
        this.missionStarted = false;
      }
      this.mapService.clearCanvas();
      const node = this.rosService.getNode("stop_mission_pub_node");
      const msg: rclnodejs.std_msgs.msg.Bool = { data: false };

      const publisher = node.createPublisher(
        "std_msgs/msg/Bool",
        "/exploration"
      );
      publisher.publish(msg);
      node.destroyPublisher(publisher);

      this.LOGGER.info("Stop mission message published");
    } catch (error) {
      this.LOGGER.err("Error stopping mission:", error);
    }
  }
}

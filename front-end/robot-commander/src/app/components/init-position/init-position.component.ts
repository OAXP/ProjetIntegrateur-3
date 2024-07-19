import { Component } from '@angular/core';
import { InfoService } from "../../services/info/info.service";
import { CommunicationService } from "../../services/communication/communication.service";
import { RobotInfo } from "../../interfaces/RobotInfo";
import { Coordinates } from "../../interfaces/Coordinates";
import { Quaternion } from 'src/app/interfaces/Quaternion';

@Component({
  selector: 'app-init-position',
  templateUrl: './init-position.component.html',
  styleUrls: ['./init-position.component.css']
})
export class InitPositionComponent {
  originRobotId: string = '';
  nonSelectedRobotPosition: Coordinates = { x: 0, y: 0 };
  nonSelectedRobotOrientation: Quaternion = { x: 0, y: 0, z: 0, w: 1 };
  private _robots: RobotInfo[];
  private isConfirmed: boolean = false;

  get confirmDisabled() {
    return this.isConfirmed || this.originRobotId === '';
  }

  constructor(private infoService: InfoService, private communicationService: CommunicationService) {
    this._robots = this.getRobots();
  }

  get nonSelectedRobot(): RobotInfo | undefined {
    // Retourne le robot non sélectionné
    return this._robots.find(robot => robot.robot_id !== this.originRobotId);
  }

  getRobots(): RobotInfo[] {
    let robots: RobotInfo[] = [];
    if (this.infoService.robotInfos.size > 0) {
      robots = Array.from(this.infoService.robotInfos.values());
    } else {
      // TODO: Remove this mock data
      robots = [
        {
          robot_id: 'limo_0', status: 'En attente de mission', battery_voltage: 10, start_time: 0.0, init_position: { x: 0, y: 0 }, init_orientation: { x: 0, y: 0, z: 0, w: 1 },
          position_x: 0,
          position_y: 0
        },
        {
          robot_id: 'limo_1', status: 'En attente de mission', battery_voltage: 10, start_time: 0.0, init_position: { x: 0, y: 0 }, init_orientation: { x: 0, y: 0, z: 0, w: 1 },
          position_x: 0,
          position_y: 0
        }
      ];
    }
    return robots;
  }

  trackByRobotId(index: number, robot: any): string {
    return robot.robot_id;
  }

  onOriginRobotChange(): void {
    // Met à jour le robot sélectionné à (0, 0)
    this.isConfirmed = false;
    const selectedRobot = this._robots.find(robot => robot.robot_id === this.originRobotId);
    if (selectedRobot) {
      selectedRobot.init_position = { x: 0, y: 0 };
      selectedRobot.init_orientation = { x: 0, y: 0, z: 0, w: 1 };
    }
  }

  updateNonSelectedRobot(): void {
    // Trouvez le robot non sélectionné et mettez à jour `nonSelectedRobotPosition`
    this.isConfirmed = false;
    const nonSelectedRobot = this.getRobots().find(robot => robot.robot_id !== this.originRobotId);
    if (nonSelectedRobot) {
      this.nonSelectedRobotPosition = { ...nonSelectedRobot.init_position };
      this.nonSelectedRobotOrientation = { ...nonSelectedRobot.init_orientation };
    }
  }

  updateNonSelectedRobotPosition(axis: 'x' | 'y', value: number): void {
    // Mettez à jour la position du robot non sélectionné
    this.isConfirmed = false;
    const nonSelectedRobot = this.nonSelectedRobot;
    if (nonSelectedRobot) {
      nonSelectedRobot.init_position[axis] = value;
    }
  }

  updateNonSelectedRobotOrientation(axis: 'x' | 'y' | 'z' | 'w', value: number): void {
    // Mettez à jour la position du robot non sélectionné
    this.isConfirmed = false;
    const nonSelectedRobot = this.nonSelectedRobot;
    if (nonSelectedRobot) {
      nonSelectedRobot.init_orientation[axis] = value;
    }
  }

  confirmInitPositions(): void {
    const initPositions = new Map(this._robots.map(robot => [robot.robot_id, { init_position: robot.init_position, init_orientation: robot.init_orientation }]));
    this.communicationService.setInitPositions(initPositions).subscribe();
    this.isConfirmed = true;
  }
  
  
}
